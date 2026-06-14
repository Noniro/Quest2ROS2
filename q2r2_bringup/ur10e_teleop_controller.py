import os
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from quest2ros.msg import OVR2ROSInputs
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
import tempfile
import ikpy.chain
import xml.etree.ElementTree as ET


def derive_chain_spec(urdf_xml, base_link='base_link', tip_link='tool0'):
    """Walk the URDF kinematic tree, returning (base_elements, mask, joint_names) for ikpy."""
    root = ET.fromstring(urdf_xml)
    children = {}
    for j in root.findall('joint'):
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        children.setdefault(parent, []).append((j.get('name'), j.get('type'), child))

    def dfs(link, path):
        if link == tip_link:
            return path
        for jname, jtype, child in children.get(link, []):
            found = dfs(child, path + [(jname, jtype, child)])
            if found is not None:
                return found
        return None

    path = dfs(base_link, [])
    if path is None:
        raise RuntimeError(f"No kinematic path from '{base_link}' to '{tip_link}' in URDF")

    base_elements = [base_link]
    mask = [False]
    joint_names = []
    for jname, jtype, child in path:
        base_elements += [jname, child]
        actuated = jtype in ('revolute', 'continuous', 'prismatic')
        mask.append(actuated)
        if actuated:
            joint_names.append(jname)
    return base_elements, mask, joint_names


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y**2 - 2*z**2,   2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x**2 - 2*z**2,  2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,        1 - 2*x**2 - 2*y**2]
    ])


def matrix_to_quaternion(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S; qx = (m[2,1]-m[1,2])/S; qy = (m[0,2]-m[2,0])/S; qz = (m[1,0]-m[0,1])/S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1]-m[1,2])/S; qx = 0.25*S; qy = (m[0,1]+m[1,0])/S; qz = (m[0,2]+m[2,0])/S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        qw = (m[0,2]-m[2,0])/S; qx = (m[0,1]+m[1,0])/S; qy = 0.25*S; qz = (m[1,2]+m[2,1])/S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        qw = (m[1,0]-m[0,1])/S; qx = (m[0,2]+m[2,0])/S; qy = (m[1,2]+m[2,1])/S; qz = 0.25*S
    return np.array([qx, qy, qz, qw])


class UR10eTeleopController(Node):
    """
    VR teleoperation controller for the Universal Robots UR10e.

    Sim mode  : publishes /joint_states so robot_state_publisher + RViz can
                visualize the commanded pose without any real hardware.
    HW mode   : publishes std_msgs/Float64MultiArray to
                /forward_position_controller/commands (ros2_control streaming).
                Mirrors real robot joint state while clutch is off.

    Clutch    : A-button (button_lower) toggle.
                On engage, the anchor is computed via own ikpy FK so the
                controller is fully self-contained (no TF dependency).
    """

    def __init__(self):
        super().__init__('ur10e_teleop_controller')
        self.get_logger().info("--- Initializing UR10e Teleop Controller ---")

        self.declare_parameter('sim_mode', True)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_joint_delta', 0.15)       # rad/step (~8.5 deg)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('fwd_pos_topic', '/forward_position_controller/commands')
        # Cartesian safety workspace (metres, robot base frame)
        self.declare_parameter('x_min', 0.15); self.declare_parameter('x_max', 1.20)
        self.declare_parameter('y_min', -0.85); self.declare_parameter('y_max', 0.85)
        self.declare_parameter('z_min', -0.20); self.declare_parameter('z_max', 1.40)

        self.sim_mode       = self.get_parameter('sim_mode').value
        self.scale_factor   = self.get_parameter('scale_factor').value
        self.publish_rate   = self.get_parameter('publish_rate').value
        self.max_joint_delta = self.get_parameter('max_joint_delta').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.fwd_pos_topic  = self.get_parameter('fwd_pos_topic').value
        self.x_bounds = (self.get_parameter('x_min').value, self.get_parameter('x_max').value)
        self.y_bounds = (self.get_parameter('y_min').value, self.get_parameter('y_max').value)
        self.z_bounds = (self.get_parameter('z_min').value, self.get_parameter('z_max').value)

        self._init_ik_chain()
        self.current_joints = {name: 0.0 for name in self.joint_names}

        # VR state
        self.allow_pose_update    = False
        self.button_lower_pressed = False
        self.first_quest_pos      = None
        self.first_quest_ori_matrix = None
        self.robot_anchor_pos     = None
        self.robot_anchor_ori_matrix = None
        self.last_pose_msg        = None

        self.pose_sub  = self.create_subscription(PoseStamped, '/q2r_right_hand_pose',   self._pose_callback,  10)
        self.input_sub = self.create_subscription(OVR2ROSInputs, '/q2r_right_hand_inputs', self._inputs_callback, 10)

        if not self.sim_mode:
            self.joint_state_sub = self.create_subscription(
                JointState, self.joint_states_topic, self._joint_state_callback,
                qos_profile_sensor_data)
            self.get_logger().info(
                f"HW mode: mirroring '{self.joint_states_topic}', "
                f"commanding '{self.fwd_pos_topic}'")
            self.fwd_pos_pub = self.create_publisher(Float64MultiArray, self.fwd_pos_topic, 10)
        else:
            self.joint_pub = self.create_publisher(JointState, self.joint_states_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        self.get_logger().info("--- UR10e Teleop Controller Ready ---")

    def _init_ik_chain(self):
        try:
            ur_description_dir = get_package_share_directory('ur_description')
            xacro_file = os.path.join(ur_description_dir, 'urdf', 'ur.urdf.xacro')
            self.get_logger().info(f"Loading UR10e URDF: {xacro_file}")

            doc = xacro.process_file(xacro_file, mappings={'ur_type': 'ur10e', 'name': 'ur'})
            urdf_xml = doc.toxml()

            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(urdf_xml)
                self.temp_urdf_path = f.name

            elements, mask, self.joint_names = derive_chain_spec(urdf_xml)
            self.active_idx = [i for i, a in enumerate(mask) if a]

            self.chain = ikpy.chain.Chain.from_urdf_file(
                self.temp_urdf_path, base_elements=elements, active_links_mask=mask)

            self.get_logger().info(
                f"IK chain loaded, joints: {self.joint_names}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IK chain: {e}")
            raise

    def _get_eef_pose(self):
        q = [0.0] * len(self.chain.links)
        for idx, name in zip(self.active_idx, self.joint_names):
            q[idx] = self.current_joints[name]
        T = self.chain.forward_kinematics(q)
        return T[:3, 3].copy(), T[:3, :3].copy()

    def _pose_callback(self, msg: PoseStamped):
        self.last_pose_msg = msg

    def _joint_state_callback(self, msg: JointState):
        if self.allow_pose_update:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joints:
                self.current_joints[name] = pos

    def _inputs_callback(self, msg: OVR2ROSInputs):
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
        if self.allow_pose_update and self.last_pose_msg is not None:
            self._process_teleoperation(self.last_pose_msg)
        self._publish_outputs()

    def _process_teleoperation(self, msg: PoseStamped):
        quest_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quest_q   = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        quest_rot = quaternion_to_matrix(quest_q)

        if self.first_quest_pos is None:
            self.first_quest_pos = quest_pos
            self.first_quest_ori_matrix = quest_rot
            self.robot_anchor_pos, self.robot_anchor_ori_matrix = self._get_eef_pose()
            self.get_logger().info(
                f"[TELEOP] Anchored. Quest: {quest_pos}, Robot EEF: {self.robot_anchor_pos}")
            return

        offset = quest_pos - self.first_quest_pos
        robot_offset = np.array([-offset[2], -offset[0], offset[1]])
        target_pos = self.robot_anchor_pos + robot_offset * self.scale_factor
        target_pos[0] = np.clip(target_pos[0], *self.x_bounds)
        target_pos[1] = np.clip(target_pos[1], *self.y_bounds)
        target_pos[2] = np.clip(target_pos[2], *self.z_bounds)

        R_vr_rel = np.dot(self.first_quest_ori_matrix.T, quest_rot)
        R_vr_to_robot = np.array([[ 0., 0., -1.], [-1., 0., 0.], [ 0., 1., 0.]])
        R_vr_rel_robot = np.dot(R_vr_to_robot, np.dot(R_vr_rel, R_vr_to_robot.T))
        target_rot = np.dot(self.robot_anchor_ori_matrix, R_vr_rel_robot)

        initial_q = [0.0] * len(self.chain.links)
        for idx, name in zip(self.active_idx, self.joint_names):
            initial_q[idx] = self.current_joints[name]

        try:
            ik_q = self.chain.inverse_kinematics(
                target_position=target_pos, target_orientation=target_rot,
                orientation_mode="all", initial_position=initial_q)

            for idx, name in zip(self.active_idx, self.joint_names):
                delta = abs(float(ik_q[idx]) - self.current_joints[name])
                if delta > self.max_joint_delta:
                    self.get_logger().warning(
                        f"[SAFETY] {name} delta {delta:.4f} rad > limit {self.max_joint_delta}. Holding.")
                    return

            for idx, name in zip(self.active_idx, self.joint_names):
                self.current_joints[name] = float(ik_q[idx])

        except Exception as e:
            self.get_logger().warning(f"IK solver failed: {e}")

    def _publish_outputs(self):
        if self.sim_mode:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name     = self.joint_names
            msg.position = [self.current_joints[n] for n in self.joint_names]
            self.joint_pub.publish(msg)

        elif self.allow_pose_update:
            # Stream positions to ros2_control forward_position_controller.
            # Joint order must match the controller's configuration (URDF order).
            cmd = Float64MultiArray()
            cmd.data = [self.current_joints[n] for n in self.joint_names]
            self.fwd_pos_pub.publish(cmd)

    def __del__(self):
        if hasattr(self, 'temp_urdf_path') and os.path.exists(self.temp_urdf_path):
            os.remove(self.temp_urdf_path)


def main(args=None):
    rclpy.init(args=args)
    node = UR10eTeleopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
