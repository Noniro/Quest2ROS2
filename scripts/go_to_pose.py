#!/usr/bin/env python3
"""
go_to_pose.py — save the robot's current joint pose, then smoothly return to it.

Use it to get a repeatable test-start pose: put the arm where you want, --save it,
then before each /go_home run this (no args) to ramp back to that exact pose.

  python3 go_to_pose.py --save        # remember where the arm is right now
  python3 go_to_pose.py               # ramp back to the saved pose
  python3 go_to_pose.py --speed 12    # same, a bit faster (deg/s)

How it moves: a slow, open-loop ramp streamed to forward_position_controller
(same safe mechanism as /go_home, just to an arbitrary saved target). No jump.

SAFETY
  - E-stop in hand, pendant speed slider low.
  - Driver up (ur10e_start.sh); forward_position_controller ACTIVE (the launch
    default — if you were testing 'scaled', switch back to fpc first).
  - The teleop node should be STOPPED so nothing else commands the arm.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FPC_TOPIC = '/forward_position_controller/commands'
DEFAULT_FILE = os.path.expanduser('~/teleop_test_pose.json')


class Mover(Node):
    def __init__(self):
        super().__init__('go_to_pose')
        self.pos = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, FPC_TOPIC, 10)

    def _cb(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def wait_for_state(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.pos is None and \
                self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pos is not None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--save', action='store_true', help='save current pose and exit')
    p.add_argument('--file', default=DEFAULT_FILE, help='where the pose is stored')
    p.add_argument('--speed', type=float, default=8.0, help='ramp speed (deg/s)')
    p.add_argument('--rate', type=float, default=30.0, help='stream rate (Hz)')
    p.add_argument('--ease', action='store_true',
                   help='cosine ease-in-out (smooth accel) instead of constant-velocity ramp')
    p.add_argument('--staggered', action='store_true',
                   help="mimic /go_home exactly: every joint at --speed, unsynchronized "
                        "finish + snap-to-target (the suspected noise profile)")
    args = p.parse_args()

    rclpy.init()
    node = Mover()
    if not node.wait_for_state():
        node.get_logger().error('No /joint_states — is the driver up (ur10e_start.sh)?')
        rclpy.shutdown(); sys.exit(1)

    cur = {j: float(node.pos[j]) for j in UR_JOINTS}

    # ---- SAVE mode ----
    if args.save:
        with open(args.file, 'w') as f:
            json.dump(cur, f, indent=2)
        print('Saved current pose (deg):')
        for j in UR_JOINTS:
            print(f'  {j:20s} {math.degrees(cur[j]):7.2f}')
        print(f'-> {args.file}')
        rclpy.shutdown(); return

    # ---- GO mode ----
    if not os.path.exists(args.file):
        node.get_logger().error(f'no saved pose at {args.file} — run with --save first.')
        rclpy.shutdown(); sys.exit(1)
    with open(args.file) as f:
        target = {j: float(v) for j, v in json.load(f).items()}

    if node.count_subscribers(FPC_TOPIC) == 0:
        node.get_logger().error(
            f'no subscriber on {FPC_TOPIC} — forward_position_controller is not active.\n'
            '  ros2 control switch_controllers '
            '--deactivate scaled_joint_trajectory_controller '
            '--activate forward_position_controller')
        rclpy.shutdown(); sys.exit(1)

    print('Returning to saved pose (deg):')
    for j in UR_JOINTS:
        print(f'  {j:20s} {math.degrees(cur[j]):7.2f}  ->  {math.degrees(target[j]):7.2f}')
    print(f'at {args.speed:.0f} deg/s. E-stop in hand. Moving in 3 s... (Ctrl-C to abort)')
    try:
        for i in (3, 2, 1):
            print(f'  {i}...'); rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print('aborted'); rclpy.shutdown(); return

    if args.staggered:
        # faithful replica of /go_home's _step_homing: each joint moves at the same
        # per-tick step (= --speed), snaps to target within tol, finishes unsynchronized.
        print(f'  (staggered/go_home-style, {args.rate:g} Hz)')
        step = math.radians(args.speed) / args.rate
        cmd = dict(cur)
        tol = 1e-4
        while rclpy.ok():
            done = True
            for j in UR_JOINTS:
                diff = target[j] - cmd[j]
                if abs(diff) < tol:
                    cmd[j] = target[j]
                else:
                    cmd[j] += max(-step, min(step, diff))
                    done = False
            msg = Float64MultiArray()
            msg.data = [cmd[j] for j in UR_JOINTS]
            node.cmd_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            if done:
                break
            time.sleep(1.0 / args.rate)
    else:
        # time-parameterized: longest-travel joint moves at --speed, all finish together
        deltas = {j: target[j] - cur[j] for j in UR_JOINTS}
        max_delta = max(abs(d) for d in deltas.values())
        total = max(max_delta / math.radians(args.speed), 1e-3)   # seconds
        n = max(1, int(total * args.rate))
        print(f'  ({"eased" if args.ease else "constant-velocity"} sync, {args.rate:g} Hz, ~{total:.1f}s)')
        for i in range(n + 1):
            a = i / n
            if args.ease:
                a = 0.5 - 0.5 * math.cos(math.pi * a)   # smooth accel/decel, no jerk
            msg = Float64MultiArray()
            msg.data = [cur[j] + a * deltas[j] for j in UR_JOINTS]
            node.cmd_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / args.rate)
    print('Reached saved pose.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
