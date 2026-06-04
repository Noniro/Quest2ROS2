import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from quest2ros.msg import OVR2ROSInputs
from tf2_ros import TransformListener, Buffer
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
import tempfile
import ikpy.chain
import os

class MotomanSimController(Node):
    def __init__(self):
        super().__init__('motoman_sim_controller')
        self.get_logger().info("--- Initializing Motoman Sim Controller Node ---")

        # Define Joint Names for Motoman SDA10F
        self.joint_names = [
            'group_3/joint_1',  # Torso waist
            'group_4/joint_1',  # Mimic torso waist
            # Left Arm
            'group_1/joint_1', 'group_1/joint_2', 'group_1/joint_3',
            'group_1/joint_4', 'group_1/joint_5', 'group_1/joint_6', 'group_1/joint_7',
            # Right Arm
            'group_2/joint_1', 'group_2/joint_2', 'group_2/joint_3',
            'group_2/joint_4', 'group_2/joint_5', 'group_2/joint_6', 'group_2/joint_7'
        ]

        # Current Joint Angles (default to 0.0)
        self.current_joints = {name: 0.0 for name in self.joint_names}

        # Anchoring and state variables for left/right hands
        self.hand_states = {
            'left': {
                'allow_pose_update': False,
                'first_quest_pos': None,
                'robot_anchor_pos': None,
                'last_pose_msg': None,
                'button_lower_pressed': False
            },
            'right': {
                'allow_pose_update': False,
                'first_quest_pos': None,
                'robot_anchor_pos': None,
                'last_pose_msg': None,
                'button_lower_pressed': False
            }
        }

        # Load URDF via Xacro and setup ikpy chains
        self._init_ik_chains()

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/q2r_left_hand_pose', lambda msg: self._pose_callback(msg, 'left'), 10)
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/q2r_right_hand_pose', lambda msg: self._pose_callback(msg, 'right'), 10)

        self.left_input_sub = self.create_subscription(
            OVR2ROSInputs, '/q2r_left_hand_inputs', lambda msg: self._inputs_callback(msg, 'left'), 10)
        self.right_input_sub = self.create_subscription(
            OVR2ROSInputs, '/q2r_right_hand_inputs', lambda msg: self._inputs_callback(msg, 'right'), 10)

        # Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to publish JointState at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)
        self.get_logger().info("--- Motoman Sim Controller Node Ready ---")

    def _init_ik_chains(self):
        try:
            # Parse Xacro to URDF XML
            support_dir = get_package_share_directory('motoman_sda10f_support')
            xacro_file = os.path.join(support_dir, 'urdf', 'sda10f.xacro')
            self.get_logger().info(f"Loading Xacro: {xacro_file}")
            
            doc = xacro.process_file(xacro_file)
            urdf_xml = doc.toxml()

            # Write to a temporary file for ikpy
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(urdf_xml)
                self.temp_urdf_path = f.name
            
            # Left Arm Kinematic Chain elements
            left_elements = [
                "base_link", "base_link_to_torso_base_link", "torso_base_link",
                "group_3/joint_1", "group_3/base_link",
                "group_1/joint_1", "group_1/link_1",
                "group_1/joint_2", "group_1/link_2",
                "group_1/joint_3", "group_1/link_3",
                "group_1/joint_4", "group_1/link_4",
                "group_1/joint_5", "group_1/link_5",
                "group_1/joint_6", "group_1/link_6",
                "group_1/joint_7", "group_1/link_7",
                "group_1/joint_7-tool0", "group_1/tool0"
            ]

            # Right Arm Kinematic Chain elements
            right_elements = [
                "base_link", "base_link_to_torso_base_link", "torso_base_link",
                "group_4/joint_1", "group_4/base_link",
                "group_2/joint_1", "group_2/link_1",
                "group_2/joint_2", "group_2/link_2",
                "group_2/joint_3", "group_2/link_3",
                "group_2/joint_4", "group_2/link_4",
                "group_2/joint_5", "group_2/link_5",
                "group_2/joint_6", "group_2/link_6",
                "group_2/joint_7", "group_2/link_7",
                "group_2/joint_7-tool0", "group_2/tool0"
            ]

            self.left_chain = ikpy.chain.Chain.from_urdf_file(self.temp_urdf_path, base_elements=left_elements)
            self.right_chain = ikpy.chain.Chain.from_urdf_file(self.temp_urdf_path, base_elements=right_elements)

            # Define active masks: fixed joints are False, group_3/joint_1 and group_4/joint_1 are False (torso is fixed at 0.0)
            self.left_chain.active_links_mask = [False, False, False, True, True, True, True, True, True, True, False]
            self.right_chain.active_links_mask = [False, False, False, True, True, True, True, True, True, True, False]

            self.get_logger().info("Successfully initialized left and right IK chains!")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize IK chains: {e}")

    def _get_eef_position_from_tf(self, side: str):
        try:
            link_name = f"group_{'1' if side == 'left' else '2'}/tool0"
            transform = self.tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
            return np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
        except Exception as e:
            self.get_logger().warning(f"Could not get transform for {side} arm tool0: {e}")
            # Fallback to default coordinates if lookup fails
            return np.array([0.1, 0.4 if side == 'left' else -0.4, 1.4])

    def _pose_callback(self, msg: PoseStamped, side: str):
        state = self.hand_states[side]
        state['last_pose_msg'] = msg

        if not state['allow_pose_update']:
            return

        # Initialize anchor if needed
        quest_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if state['first_quest_pos'] is None:
            state['first_quest_pos'] = quest_pos
            state['robot_anchor_pos'] = self._get_eef_position_from_tf(side)
            self.get_logger().info(f"[{side.upper()} Arm] Anchored. Quest start: {quest_pos}, Robot start: {state['robot_anchor_pos']}")
            return

        # Compute translation offset in VR space (X=right, Y=up, Z=backward)
        offset = quest_pos - state['first_quest_pos']

        # Transform offset from VR space to Robot base space (X=forward, Y=left, Z=up)
        robot_offset = np.array([
            -offset[2],  # Robot X (Forward) = -Quest Z
            -offset[0],  # Robot Y (Left) = -Quest X
            offset[1]    # Robot Z (Up) = Quest Y
        ])

        # Apply scaling (1.0 means 1:1 mapping, can be increased to amplify reach)
        scale_factor = 1.2
        target_pos = state['robot_anchor_pos'] + robot_offset * scale_factor

        # Solve IK using ikpy with temporal consistency (using current joints as initial guess)
        try:
            if side == 'left':
                initial_q = [0.0] * len(self.left_chain.links)
                for i in range(7):
                    initial_q[i+3] = self.current_joints[f'group_1/joint_{i+1}']
                
                ik_q = self.left_chain.inverse_kinematics(target_position=target_pos, initial_position=initial_q)
                # Map back to our joints dict
                for i in range(7):
                    self.current_joints[f'group_1/joint_{i+1}'] = float(ik_q[i+3])
            else:
                initial_q = [0.0] * len(self.right_chain.links)
                for i in range(7):
                    initial_q[i+3] = self.current_joints[f'group_2/joint_{i+1}']

                ik_q = self.right_chain.inverse_kinematics(target_position=target_pos, initial_position=initial_q)
                # Map back to our joints dict
                for i in range(7):
                    self.current_joints[f'group_2/joint_{i+1}'] = float(ik_q[i+3])
        except Exception as e:
            self.get_logger().warn(f"IK solver error on {side} arm: {e}")

    def _inputs_callback(self, msg: OVR2ROSInputs, side: str):
        state = self.hand_states[side]

        # Lower button toggles enable/disable pose updates and resets anchors
        if msg.button_lower and not state['button_lower_pressed']:
            state['allow_pose_update'] = not state['allow_pose_update']
            state['first_quest_pos'] = None
            state['robot_anchor_pos'] = None
            status = "ENABLED" if state['allow_pose_update'] else "DISABLED"
            self.get_logger().info(f"[{side.upper()} Arm] Lower button pressed. Pose streaming: {status}")

        state['button_lower_pressed'] = msg.button_lower

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_joints[name] for name in self.joint_names]
        self.joint_pub.publish(msg)

    def __del__(self):
        # Cleanup temp file
        if hasattr(self, 'temp_urdf_path') and os.path.exists(self.temp_urdf_path):
            os.remove(self.temp_urdf_path)

def main(args=None):
    rclpy.init(args=args)
    node = MotomanSimController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
