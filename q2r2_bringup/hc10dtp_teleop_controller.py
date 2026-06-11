import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from quest2ros.msg import OVR2ROSInputs
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
import tempfile
import ikpy.chain

# --- Math Helpers ---
def quaternion_to_matrix(q):
    # q is [x, y, z, w]
    x, y, z, w = q
    return np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x**2 - 2*y**2]
    ])

def matrix_to_quaternion(m):
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m[2,1] - m[1,2]) / S
        qy = (m[0,2] - m[2,0]) / S
        qz = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        qw = (m[0,2] - m[2,0]) / S
        qx = (m[0,1] + m[1,0]) / S
        qy = 0.25 * S
        qz = (m[1,2] + m[2,1]) / S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        qw = (m[1,0] - m[0,1]) / S
        qx = (m[0,2] + m[2,0]) / S
        qy = (m[1,2] + m[2,1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])


class HC10DTPTeleopController(Node):
    def __init__(self):
        super().__init__('hc10dtp_teleop_controller')
        self.get_logger().info("--- Initializing Yaskawa HC10DTP Teleop Controller ---")

        # Declare parameters
        self.declare_parameter('sim_mode', True)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_joint_delta', 0.15)  # rad per step (~8.5 deg)

        # Topic names are parameterized so the node can target a namespaced
        # MotoROS2 instance (e.g. /our_hc10/joint_states) and never cross wires
        # with another robot on the same DDS domain.
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('trajectory_topic', '/joint_trajectory_controller/joint_trajectory')
        
        # Bounding box parameters (Safety workspace bounds relative to robot base)
        self.declare_parameter('x_min', 0.15)
        self.declare_parameter('x_max', 0.95)
        self.declare_parameter('y_min', -0.75)
        self.declare_parameter('y_max', 0.75)
        self.declare_parameter('z_min', -0.15)
        self.declare_parameter('z_max', 1.25)

        # Read parameters
        self.sim_mode = self.get_parameter('sim_mode').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_joint_delta = self.get_parameter('max_joint_delta').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        
        self.x_bounds = (self.get_parameter('x_min').value, self.get_parameter('x_max').value)
        self.y_bounds = (self.get_parameter('y_min').value, self.get_parameter('y_max').value)
        self.z_bounds = (self.get_parameter('z_min').value, self.get_parameter('z_max').value)

        # Define Joint Names for HC10DTP
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        # Initialize joint angles dictionary (defaults to 0.0)
        self.current_joints = {name: 0.0 for name in self.joint_names}

        # VR State variables
        self.allow_pose_update = False
        self.button_lower_pressed = False
        self.first_quest_pos = None
        self.first_quest_ori_matrix = None
        
        self.robot_anchor_pos = None
        self.robot_anchor_ori_matrix = None
        self.last_pose_msg = None

        # Load URDF dynamically & initialize ikpy chain
        self._init_ik_chain()

        # VR Right Hand Subscribers (typically right hand matches single arm teleop)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/q2r_right_hand_pose', self._pose_callback, 10)
        self.input_sub = self.create_subscription(
            OVR2ROSInputs, '/q2r_right_hand_inputs', self._inputs_callback, 10)

        # HW mode: track the real robot state published by MotoROS2 so anchoring
        # and IK seeding always start from the robot's true configuration.
        if not self.sim_mode:
            self.joint_state_sub = self.create_subscription(
                JointState, self.joint_states_topic, self._joint_state_callback,
                qos_profile_sensor_data)
            self.get_logger().info(
                f"HW mode: mirroring '{self.joint_states_topic}', "
                f"commanding '{self.trajectory_topic}'")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.trajectory_topic, 10)

        # Main Teleoperation Timer Loop
        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        self.get_logger().info("--- Yaskawa HC10DTP Teleop Controller Ready ---")

    def _init_ik_chain(self):
        try:
            # Parse Xacro to URDF XML
            support_dir = get_package_share_directory('motoman_hc10_support')
            xacro_file = os.path.join(support_dir, 'urdf', 'hc10dtp_b00.xacro')
            self.get_logger().info(f"Loading Xacro: {xacro_file}")
            
            doc = xacro.process_file(xacro_file)
            urdf_xml = doc.toxml()

            # Write to temporary file for ikpy
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(urdf_xml)
                self.temp_urdf_path = f.name
            
            # Chain elements from base_link to tool0
            elements = [
                "base_link",
                "joint_1", "link_1",
                "joint_2", "link_2",
                "joint_3", "link_3",
                "joint_4", "link_4",
                "joint_5", "link_5",
                "joint_6", "link_6",
                "joint_6-flange", "flange",
                "flange-tool0", "tool0"
            ]

            # Define active mask (fixed joints are set to False)
            # Link index mapping:
            # 0: Base link (fixed) -> False
            # 1: joint_1 (revolute) -> True
            # 2: joint_2 (revolute) -> True
            # 3: joint_3 (revolute) -> True
            # 4: joint_4 (revolute) -> True
            # 5: joint_5 (revolute) -> True
            # 6: joint_6 (revolute) -> True
            # 7: joint_6-flange (fixed) -> False
            # 8: flange-tool0 (fixed) -> False
            mask = [False, True, True, True, True, True, True, False, False]

            self.chain = ikpy.chain.Chain.from_urdf_file(
                self.temp_urdf_path,
                base_elements=elements,
                active_links_mask=mask
            )

            self.get_logger().info("Successfully loaded HC10DTP IK Chain!")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize IK chain: {e}")

    def _get_eef_pose(self):
        # FK of the internal joint state via the same ikpy chain used for IK,
        # so the anchor is always consistent with the solver (no TF dependency).
        q = [0.0] * len(self.chain.links)
        for i, name in enumerate(self.joint_names):
            q[i + 1] = self.current_joints[name]
        T = self.chain.forward_kinematics(q)
        return T[:3, 3].copy(), T[:3, :3].copy()

    def _pose_callback(self, msg: PoseStamped):
        self.last_pose_msg = msg

    def _joint_state_callback(self, msg: JointState):
        # Mirror the real robot while the clutch is disengaged; during active
        # streaming our own commands are the reference.
        if self.allow_pose_update:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joints:
                self.current_joints[name] = pos

    def _inputs_callback(self, msg: OVR2ROSInputs):
        # The lower button (A button) behaves as the clutch toggle
        if msg.button_lower and not self.button_lower_pressed:
            self.allow_pose_update = not self.allow_pose_update
            self.first_quest_pos = None
            self.first_quest_ori_matrix = None
            self.robot_anchor_pos = None
            self.robot_anchor_ori_matrix = None
            status = "ENABLED" if self.allow_pose_update else "DISABLED"
            self.get_logger().info(f"[CLUTCH] Pose teleoperation: {status}")

        self.button_lower_pressed = msg.button_lower

    def _timer_callback(self):
        # Handle state publishing in both simulation and active teleop
        if self.allow_pose_update and self.last_pose_msg is not None:
            self._process_teleoperation(self.last_pose_msg)
        
        # Publish current joint values to ROS
        self._publish_outputs()

    def _process_teleoperation(self, msg: PoseStamped):
        # Extract Quest spatial position and orientation
        quest_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quest_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        quest_rot = quaternion_to_matrix(quest_q)

        # 1. Initialize anchors when streaming starts
        if self.first_quest_pos is None:
            self.first_quest_pos = quest_pos
            self.first_quest_ori_matrix = quest_rot
            self.robot_anchor_pos, self.robot_anchor_ori_matrix = self._get_eef_pose()
            self.get_logger().info(f"[TELEOP] Anchored. Quest pos: {quest_pos}, Robot pos: {self.robot_anchor_pos}")
            return

        # 2. Compute relative spatial offset in VR space
        offset = quest_pos - self.first_quest_pos

        # Mapping axes: Robot X = -Quest Z, Robot Y = -Quest X, Robot Z = Quest Y
        robot_offset = np.array([
            -offset[2],  # Robot X (Forward)
            -offset[0],  # Robot Y (Left)
            offset[1]    # Robot Z (Up)
        ])

        # Apply spatial scaling to amplify reach
        target_pos = self.robot_anchor_pos + robot_offset * self.scale_factor

        # 3. Apply Cartesian safety workspace bounds
        target_pos[0] = np.clip(target_pos[0], self.x_bounds[0], self.x_bounds[1])
        target_pos[1] = np.clip(target_pos[1], self.y_bounds[0], self.y_bounds[1])
        target_pos[2] = np.clip(target_pos[2], self.z_bounds[0], self.z_bounds[1])

        # 4. Compute relative orientation changes
        # VR relative rotation: R_vr_rel = R_vr_anchor_inv * R_vr
        R_vr_rel = np.dot(self.first_quest_ori_matrix.T, quest_rot)

        # Convert relative orientation change to Robot space using the VR-to-Robot mapping matrix
        # R_vr_to_robot mapping:
        # X_robot = -Z_vr => [0, 0, -1]
        # Y_robot = -X_vr => [-1, 0, 0]
        # Z_robot = Y_vr  => [0, 1, 0]
        R_vr_to_robot = np.array([
            [ 0.0, 0.0, -1.0],
            [-1.0, 0.0,  0.0],
            [ 0.0, 1.0,  0.0]
        ])

        # R_vr_rel_robot = R_vr_to_robot * R_vr_rel * R_vr_to_robot^T
        R_vr_rel_robot = np.dot(R_vr_to_robot, np.dot(R_vr_rel, R_vr_to_robot.T))

        # Absolute robot target orientation: R_target = R_robot_anchor * R_vr_rel_robot
        target_rot = np.dot(self.robot_anchor_ori_matrix, R_vr_rel_robot)

        # 5. Solve IK using ikpy with temporal consistency (seeding with current joints)
        initial_q = [0.0] * len(self.chain.links)
        for i, name in enumerate(self.joint_names):
            initial_q[i + 1] = self.current_joints[name]

        try:
            ik_q = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_rot,
                orientation_mode="all",
                initial_position=initial_q
            )

            # 6. Apply software safety checks (velocity delta clamp)
            valid_solution = True
            for i, name in enumerate(self.joint_names):
                new_val = float(ik_q[i + 1])
                old_val = self.current_joints[name]
                if abs(new_val - old_val) > self.max_joint_delta:
                    self.get_logger().warning(
                        f"[SAFETY] Joint delta command rejected! {name} jumped by {abs(new_val - old_val):.4f} rad "
                        f"(threshold: {self.max_joint_delta} rad). Holding position."
                    )
                    valid_solution = False
                    break

            if valid_solution:
                # Update current joints dictionary
                for i, name in enumerate(self.joint_names):
                    self.current_joints[name] = float(ik_q[i + 1])

        except Exception as e:
            self.get_logger().warning(f"IK Solver failed to converge: {e}")

    def _publish_outputs(self):
        # 1. Publish JointState in sim mode only; in HW mode MotoROS2 owns
        #    /joint_states and publishing here would fight the real robot state.
        if self.sim_mode:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = [self.current_joints[name] for name in self.joint_names]
            self.joint_pub.publish(joint_state_msg)

        # 2. Publish JointTrajectory (command real motors when not in sim_mode and clutch is active)
        if not self.sim_mode and self.allow_pose_update:
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = [self.current_joints[name] for name in self.joint_names]
            # Command trajectory convergence in 40ms to keep smooth stream
            point.time_from_start = Duration(sec=0, nanosec=40000000)
            
            traj_msg.points = [point]
            self.trajectory_pub.publish(traj_msg)

    def __del__(self):
        if hasattr(self, 'temp_urdf_path') and os.path.exists(self.temp_urdf_path):
            os.remove(self.temp_urdf_path)


def main(args=None):
    rclpy.init(args=args)
    node = HC10DTPTeleopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
