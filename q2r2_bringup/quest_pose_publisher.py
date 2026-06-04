#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from quest2ros.msg import OVR2ROSInputs
from tf2_ros import TransformListener, Buffer
import numpy as np
import tf_transformations
import socket
import json
from collections import deque

class QuestArmTracker:
    def __init__(self, node, side, udp_sock, udp_addr):
        self.node = node
        self.side = side
        self.udp_sock = udp_sock
        self.udp_addr = udp_addr

        self.allow_pose_update = False
        self.button_lower_pressed = False
        
        self.first_quest_pos = None
        self.first_quest_ori = None

        self.pos_history = deque(maxlen=node.filter_window_size)
        self.ori_history = deque(maxlen=node.filter_window_size)

        # --- CONTROLLER ORIENTATION ALIGNMENT OFFSETS (In Degrees) ---
        # Adjust these to comfortably align your hand posture with the robot gripper
        if self.side == 'left':
            self.roll_offset = 0.0
            self.pitch_offset = 0.0
            self.yaw_offset = 0.0  # Face right by default
        else:
            self.roll_offset = 0.0
            self.pitch_offset = 0.0
            self.yaw_offset = 0.0   # Face right by default

        # Convert Euler offsets to Quaternion
        r_rad = np.radians(self.roll_offset)
        p_rad = np.radians(self.pitch_offset)
        y_rad = np.radians(self.yaw_offset)
        self.q_offset = tf_transformations.quaternion_from_euler(r_rad, p_rad, y_rad)

        # Selected respective alignment quaternions based on side
        if self.side == 'left':
            self.q_vr_to_robot = self.node.q_vr_to_robot_left
            self.q_vr_to_robot_inv = self.node.q_vr_to_robot_left_inv
        else:
            self.q_vr_to_robot = self.node.q_vr_to_robot_right
            self.q_vr_to_robot_inv = self.node.q_vr_to_robot_right_inv

        # Subscriptions
        self.pose_sub = node.create_subscription(
            PoseStamped, f'/q2r_{side}_hand_pose', self._pose_callback, 10)
        self.input_sub = node.create_subscription(
            OVR2ROSInputs, f'/q2r_{side}_hand_inputs', self._inputs_callback, 10)

        # Filtered Pose Publisher (for ROS 2 visibility)
        self.filtered_pose_pub = node.create_publisher(
            PoseStamped, f'/q2r_{side}_hand_pose_filtered', 10)

        self.last_toggle_time = 0.0

    def _get_eef_pose_from_tf(self):
        try:
            # Try looking up the TF (will fallback if not bridged)
            link_name = f'arm_{self.side}_link_tool0'
            transform = self.node.tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
            pos = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            ori = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            return pos, ori
        except Exception:
            # Safe offline fallback coordinates
            y_sign = 1.0 if self.side == 'left' else -1.0
            pos_fallback = np.array([0.5, y_sign * 0.4, 1.2])
            ori_fallback = np.array([0.0, 1.0, 0.0, 0.0]) # Facing down
            return pos_fallback, ori_fallback

    def _apply_filter(self, pos, ori):
        self.pos_history.append(pos)
        
        if len(self.ori_history) > 0:
            # Align quaternion hemisphere to prevent cancellation during averaging
            if np.dot(ori, self.ori_history[-1]) < 0.0:
                ori = -ori
        self.ori_history.append(ori)

        avg_pos = np.mean(self.pos_history, axis=0)

        avg_ori = np.mean(self.ori_history, axis=0)
        norm = np.linalg.norm(avg_ori)
        if norm > 1e-9:
            avg_ori /= norm
        else:
            avg_ori = ori
        
        return avg_pos, avg_ori

    def _inputs_callback(self, msg: OVR2ROSInputs):
        # Log button presses for debugging
        if msg.button_lower or msg.button_upper or msg.press_index > 0.5:
            self.node.get_logger().info(
                f"[INPUT - {self.side.upper()}] lower={msg.button_lower}, upper={msg.button_upper}, index={msg.press_index:.2f}"
            )

        current_time = self.node.get_clock().now().nanoseconds / 1e9

        # If tracking is disabled, upper button (B/Y) triggers homing
        if not self.allow_pose_update and msg.button_upper and not getattr(self, 'button_upper_pressed_last', False):
            if current_time - self.last_toggle_time > 0.3:
                self.last_toggle_time = current_time
                self.node.get_logger().info(f"[TELEOP - {self.side.upper()}] Homing requested via B/Y button.")
                payload = {
                    "side": self.side,
                    "active": False,
                    "home": True
                }
                try:
                    self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
                except Exception as e:
                    self.node.get_logger().error(f"Failed to send UDP home packet: {e}")
        self.button_upper_pressed_last = msg.button_upper

        # The lower button on the controller (A for right, X for left)
        if msg.button_lower and not self.button_lower_pressed:
            if current_time - self.last_toggle_time > 0.3:
                self.allow_pose_update = not self.allow_pose_update
                self.first_quest_pos = None
                self.first_quest_ori = None
                self.last_toggle_time = current_time
                status = "ENABLED" if self.allow_pose_update else "DISABLED"
                self.node.get_logger().info(f"[TELEOP - {self.side.upper()}] Tracking: {status}")
                if not self.allow_pose_update:
                    # Notify client that tracking is disabled
                    payload = {
                        "side": self.side,
                        "active": False,
                        "home": False
                    }
                    self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
            else:
                self.node.get_logger().info(f"[TELEOP - {self.side.upper()}] Debounced button press.")
            
        self.button_lower_pressed = msg.button_lower

    def _pose_callback(self, msg: PoseStamped):
        # Periodically print that we are receiving poses to verify streaming
        if not hasattr(self, '_pose_count'):
            self._pose_count = 0
        self._pose_count += 1
        if self._pose_count % 90 == 0:
            self.node.get_logger().info(
                f"[POSE - {self.side.upper()}] Recv raw pos: [{msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}], tracking_allowed={self.allow_pose_update}"
            )

        if not self.allow_pose_update:
            # Send inactive packet over UDP periodically to ensure client knows we are inactive
            if not hasattr(self, '_last_inactive_send_time'):
                self._last_inactive_send_time = 0.0
            current_time = self.node.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_inactive_send_time > 0.1: # 10Hz
                self._last_inactive_send_time = current_time
                payload = {
                    "side": self.side,
                    "active": False,
                    "anchor": False,
                    "home": False
                }
                try:
                    self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
                except Exception as e:
                    pass
            return

        # Extract raw position and orientation
        raw_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        raw_ori = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        # Filter input coordinates
        filtered_vr_pos, filtered_vr_ori = self._apply_filter(raw_pos, raw_ori)

        # Anchor tracking on first frame of enabling
        if self.first_quest_pos is None:
            self.first_quest_pos = filtered_vr_pos
            self.first_quest_ori = filtered_vr_ori
            self.node.get_logger().info(f"[TELEOP - {self.side.upper()}] Anchored VR pose: {self.first_quest_pos}")
            # Send initial packet to notify client to anchor
            payload = {
                "side": self.side,
                "active": True,
                "anchor": True,
                "x": 0.0, "y": 0.0, "z": 0.0,
                "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
            }
            self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
            return

        # Compute relative Cartesian offset in VR space
        offset = filtered_vr_pos - self.first_quest_pos

        # Map offset directly (1st -> X_robot, 2nd -> Y_robot, 3rd -> Z_robot)
        robot_offset = np.array([
            offset[0],
            offset[1],
            offset[2]
        ])

        # Compute relative rotation in VR space
        q_quest_initial_inv = tf_transformations.quaternion_inverse(self.first_quest_ori)
        q_quest_relative = tf_transformations.quaternion_multiply(filtered_vr_ori, q_quest_initial_inv)

        # Map relative rotation from VR frame to Robot base frame
        q_robot_relative = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(self.q_vr_to_robot, q_quest_relative),
            self.q_vr_to_robot_inv
        )

        # Apply static orientation alignment offset
        q_robot_relative = tf_transformations.quaternion_multiply(q_robot_relative, self.q_offset)

        # Publish PoseStamped locally (ROS 2) for visualization
        out_msg = PoseStamped()
        out_msg.header.stamp = self.node.get_clock().now().to_msg()
        out_msg.header.frame_id = 'base_link'
        out_msg.pose.position.x = float(robot_offset[0])
        out_msg.pose.position.y = float(robot_offset[1])
        out_msg.pose.position.z = float(robot_offset[2])
        out_msg.pose.orientation.x = float(q_robot_relative[0])
        out_msg.pose.orientation.y = float(q_robot_relative[1])
        out_msg.pose.orientation.z = float(q_robot_relative[2])
        out_msg.pose.orientation.w = float(q_robot_relative[3])
        self.filtered_pose_pub.publish(out_msg)

        # Send target pose to ROS 1 over UDP
        payload = {
            "side": self.side,
            "active": True,
            "anchor": False,
            "x": float(robot_offset[0] * self.node.scale_factor),
            "y": float(robot_offset[1] * self.node.scale_factor),
            "z": float(robot_offset[2] * self.node.scale_factor),
            "qx": float(q_robot_relative[0]),
            "qy": float(q_robot_relative[1]),
            "qz": float(q_robot_relative[2]),
            "qw": float(q_robot_relative[3])
        }
        try:
            self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
        except Exception as e:
            self.node.get_logger().error(f"Failed to send UDP packet ({self.side}): {e}")

class QuestPosePublisher(Node):
    def __init__(self):
        super().__init__('quest_pose_publisher')
        self.get_logger().info("--- Initializing Dual Quest Pose Publisher Node ---")

        # Declare parameters
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('udp_port', 5005)

        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.udp_port = self.get_parameter('udp_port').value

        # Left side VR -> Robot rotation mapping (Yaw to Green, normal pitch)
        R_vr_to_robot_left = np.array([
            [1.0, 0.0, 0.0],  # X_msg -> X_local (Red)
            [0.0, 0.0, 1.0],  # Z_msg -> Y_local (Green)
            [0.0, 1.0, 0.0]   # Y_msg -> Z_local (Blue)
        ])
        M_left = np.eye(4)
        M_left[0:3, 0:3] = R_vr_to_robot_left
        self.q_vr_to_robot_left = tf_transformations.quaternion_from_matrix(M_left)
        self.q_vr_to_robot_left_inv = tf_transformations.quaternion_inverse(self.q_vr_to_robot_left)

        # Right side VR -> Robot rotation mapping (Yaw to Green, inverted pitch)
        R_vr_to_robot_right = np.array([
            [1.0, 0.0, 0.0],  # X_msg -> X_local (Red)
            [0.0, 0.0, 1.0],  # Z_msg -> Y_local (Green)
            [0.0, -1.0, 0.0]  # -Y_msg -> Z_local (Blue)  (Inverts Pitch!)
        ])
        M_right = np.eye(4)
        M_right[0:3, 0:3] = R_vr_to_robot_right
        self.q_vr_to_robot_right = tf_transformations.quaternion_from_matrix(M_right)
        self.q_vr_to_robot_right_inv = tf_transformations.quaternion_inverse(self.q_vr_to_robot_right)

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # UDP Socket Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = ("127.0.0.1", self.udp_port)

        # Initialize Trackers for both sides
        self.left_tracker = QuestArmTracker(self, 'left', self.sock, self.udp_addr)
        self.right_tracker = QuestArmTracker(self, 'right', self.sock, self.udp_addr)

        self.get_logger().info(f"Dual Quest Pose Publisher ready. Bridging to UDP port {self.udp_port}.")

def main(args=None):
    rclpy.init(args=args)
    node = QuestPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
