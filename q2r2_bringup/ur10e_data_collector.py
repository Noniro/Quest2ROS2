"""
UR10e episode data collector for World Action Model training.

Records synchronized episodes (observation + action streams) to rosbag2.
An episode starts when the VR clutch is engaged and ends when it is released.
Each episode is saved as a separate bag in ~/teleop_data/<timestamp>/.

Topics recorded per episode:
  /camera/camera/color/image_raw       — RGB observation (640x480 @ 30fps)
  /camera/camera/depth/image_rect_raw  — Aligned depth
  /camera/camera/imu                   — D435i IMU
  /joint_states                        — Robot proprioception
  /q2r_right_hand_pose                 — VR EEF target (the "action")
  /q2r_right_hand_inputs               — Clutch + button state

Rosbag2 bags can be replayed, converted to HDF5, or fed directly into
lerobot / Cosmos data pipelines via the standard rosbags library.
"""
import os
import time
import datetime
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState, Image, Imu
from geometry_msgs.msg import PoseStamped
from quest2ros.msg import OVR2ROSInputs
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    import rosbag2_py
    _ROSBAG2_AVAILABLE = True
except ImportError:
    _ROSBAG2_AVAILABLE = False


_RECORD_TOPICS = {
    '/camera/camera/color/image_raw':      'sensor_msgs/msg/Image',
    '/camera/camera/depth/image_rect_raw': 'sensor_msgs/msg/Image',
    '/camera/camera/imu':                  'sensor_msgs/msg/Imu',
    '/joint_states':                       'sensor_msgs/msg/JointState',
    '/q2r_right_hand_pose':                'geometry_msgs/msg/PoseStamped',
    '/q2r_right_hand_inputs':              'quest2ros/msg/OVR2ROSInputs',
}

_CAMERA_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class UR10eDataCollector(Node):
    def __init__(self):
        super().__init__('ur10e_data_collector')
        self.declare_parameter('data_dir', os.path.expanduser('~/teleop_data'))
        self.data_dir = self.get_parameter('data_dir').value
        os.makedirs(self.data_dir, exist_ok=True)

        if not _ROSBAG2_AVAILABLE:
            self.get_logger().error(
                "rosbag2_py not available — install ros-humble-rosbag2-py. "
                "Data collection is disabled.")

        self._recording   = False
        self._writer      = None
        self._episode_idx = 0
        self._button_prev = False
        self._msg_buffer  = {}  # topic -> list of (timestamp_ns, serialized)

        self._subs = []
        self._subs.append(self.create_subscription(
            OVR2ROSInputs, '/q2r_right_hand_inputs', self._inputs_cb, 10))
        self._subs.append(self.create_subscription(
            JointState, '/joint_states', self._make_cb('/joint_states'), qos_profile_sensor_data))
        self._subs.append(self.create_subscription(
            PoseStamped, '/q2r_right_hand_pose', self._make_cb('/q2r_right_hand_pose'), 10))
        self._subs.append(self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self._make_cb('/camera/camera/color/image_raw'), _CAMERA_QOS))
        self._subs.append(self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self._make_cb('/camera/camera/depth/image_rect_raw'), _CAMERA_QOS))
        self._subs.append(self.create_subscription(
            Imu, '/camera/camera/imu',
            self._make_cb('/camera/camera/imu'), _CAMERA_QOS))

        self.get_logger().info(
            f"Data collector ready. Episodes → {self.data_dir}")

    def _make_cb(self, topic):
        def cb(msg):
            if not self._recording or self._writer is None:
                return
            ts = self.get_clock().now().nanoseconds
            try:
                self._writer.write(topic, serialize_message(msg), ts)
            except Exception as e:
                self.get_logger().warning(f"Write error on {topic}: {e}")
        return cb

    def _inputs_cb(self, msg: OVR2ROSInputs):
        pressed = msg.button_lower
        rising  = pressed and not self._button_prev
        falling = not pressed and self._button_prev
        self._button_prev = pressed

        if rising:
            self._start_episode(msg)
        elif falling and self._recording:
            self._stop_episode()

        # Always forward to the recording buffer too
        if self._recording and self._writer is not None:
            ts = self.get_clock().now().nanoseconds
            try:
                self._writer.write('/q2r_right_hand_inputs', serialize_message(msg), ts)
            except Exception as e:
                self.get_logger().warning(f"Write error: {e}")

    def _start_episode(self, _msg):
        if not _ROSBAG2_AVAILABLE:
            return
        self._episode_idx += 1
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(self.data_dir, f"episode_{self._episode_idx:04d}_{stamp}")
        os.makedirs(bag_path, exist_ok=True)

        storage_opts = rosbag2_py._storage.StorageOptions(uri=bag_path, storage_id='sqlite3')
        conv_opts    = rosbag2_py._storage.ConverterOptions('', '')
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(storage_opts, conv_opts)

        for topic, msg_type in _RECORD_TOPICS.items():
            self._writer.create_topic(rosbag2_py._storage.TopicMetadata(
                name=topic, type=msg_type, serialization_format='cdr'))

        self._recording = True
        self.get_logger().info(
            f"[REC ▶] Episode {self._episode_idx} → {bag_path}")

    def _stop_episode(self):
        self._recording = False
        if self._writer is not None:
            del self._writer   # flushes and closes
            self._writer = None
        self.get_logger().info(
            f"[REC ■] Episode {self._episode_idx} saved.")

    def destroy_node(self):
        if self._recording:
            self._stop_episode()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UR10eDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
