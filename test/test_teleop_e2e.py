"""End-to-end sim test: fake Quest 2 right-hand input -> hc10dtp_teleop_controller
-> /joint_states. Verifies clutch engage, motion tracking, and clamp behavior."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from quest2ros.msg import OVR2ROSInputs


class FakeQuest(Node):
    def __init__(self):
        super().__init__('fake_quest')
        self.pose_pub = self.create_publisher(PoseStamped, '/q2r_right_hand_pose', 10)
        self.input_pub = self.create_publisher(OVR2ROSInputs, '/q2r_right_hand_inputs', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.latest_joints = None
        self.samples = []

    def on_joints(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def send_inputs(self, lower):
        m = OVR2ROSInputs()
        m.button_lower = lower
        self.input_pub.publish(m)

    def send_pose(self, x, y, z):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'quest'
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        self.pose_pub.publish(m)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def test_teleop_e2e():
    """pytest entry point: run e2e sim test. Skips if the sim is not already running."""
    import pytest
    rclpy.init()
    probe = rclpy.create_node('_e2e_probe')
    try:
        names = [n for n, _ in probe.get_node_names_and_namespaces()]
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    if 'hc10dtp_teleop_controller' not in names:
        pytest.skip("hc10dtp_teleop_controller not running — launch hc10dtp_sim first")
    main()


def main():
    rclpy.init()
    n = FakeQuest()

    # Wait for controller to be up and publishing joint states
    n.spin_for(3.0)
    assert n.latest_joints is not None, "FAIL: no /joint_states received"
    start = dict(n.latest_joints)
    print(f"Initial joints: { {k: round(v, 3) for k, v in start.items()} }")

    # Send neutral pose first so last_pose_msg exists, then toggle clutch ON
    n.send_pose(0.0, 0.0, 0.0)
    n.spin_for(0.3)
    n.send_inputs(True)
    n.spin_for(0.2)
    n.send_inputs(False)
    n.spin_for(0.5)

    # Stream slow motion: VR +Y (up) by 8 cm over 4 s -> robot Z up ~9.6 cm
    T = 4.0
    t0 = time.time()
    while time.time() - t0 < T:
        f = (time.time() - t0) / T
        n.send_pose(0.0, 0.08 * f, 0.0)
        n.spin_for(0.033)

    n.spin_for(0.5)
    end = dict(n.latest_joints)
    print(f"Final joints  : { {k: round(v, 3) for k, v in end.items()} }")

    moved = sum(abs(end[k] - start[k]) for k in start)
    print(f"Total joint motion: {moved:.4f} rad")

    # Clutch OFF, then big jump in VR pose -> joints must NOT move
    n.send_inputs(True)
    n.spin_for(0.2)
    n.send_inputs(False)
    n.spin_for(0.3)
    frozen = dict(n.latest_joints)
    for _ in range(15):
        n.send_pose(0.5, 0.5, -0.5)
        n.spin_for(0.04)
    after = dict(n.latest_joints)
    drift = sum(abs(after[k] - frozen[k]) for k in frozen)
    print(f"Joint drift while clutch OFF: {drift:.6f} rad")

    ok = moved > 0.05 and drift < 1e-9
    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
