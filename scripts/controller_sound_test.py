#!/usr/bin/env python3
"""
controller_sound_test.py — diagnose the control-box noise by running the SAME
relative point-to-point move through different controllers, so the only thing that
changes is HOW the motion is delivered.

Two delivery modes (`--controller`):
  scaled : send a timed trajectory to scaled_joint_trajectory_controller
           (interpolated at the 500 Hz servo rate — the MoveIt / pendant path).
  fpc    : stream interpolated position snapshots to forward_position_controller
           at --rate Hz (the teleop / go_home path — zero-order hold between ticks).

So you can A/B/C the exact same shoulder move:
  fpc @ 30 Hz   vs   fpc @ 125 Hz   vs   scaled
and hear which one clicks.

SAFETY
  - Slow, small, RELATIVE moves only. E-stop in hand, pendant speed slider low.
  - Run with the UR driver up (ur10e_start.sh) and the teleop node STOPPED (this
    script is the only thing commanding the arm — avoids two publishers fighting).
  - The matching controller must be ACTIVE:
      fpc    -> forward_position_controller        (the launch default)
      scaled -> scaled_joint_trajectory_controller (switch to it first)
    The script checks and tells you the switch command if the wrong one is active.

Examples
  python3 controller_sound_test.py --list
  python3 controller_sound_test.py --controller fpc    --rate 30  --joint shoulder_lift_joint --deg 10 --secs 5
  python3 controller_sound_test.py --controller fpc    --rate 125 --joint shoulder_lift_joint --deg 10 --secs 5
  python3 controller_sound_test.py --controller scaled            --joint shoulder_lift_joint --deg 10 --secs 5
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
FPC_TOPIC = '/forward_position_controller/commands'

MAX_DEG = 45.0
MIN_SECS = 0.8


class Tester(Node):
    def __init__(self):
        super().__init__('controller_sound_test')
        self.pos = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.ac = ActionClient(self, FollowJointTrajectory, ACTION)
        self.cmd_pub = self.create_publisher(Float64MultiArray, FPC_TOPIC, 10)

    def _cb(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def wait_for_state(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.pos is None and \
                self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pos is not None


def make_duration(t):
    d = Duration()
    d.sec = int(t)
    d.nanosec = int((t - int(t)) * 1e9)
    return d


def build_waypoints(start, joints, delta, secs, cycles, complex_mode):
    """(time, full-6-joint dict) list: out-and-back on the given joints, t starts at 0.
    complex_mode alternates the sign per joint so the joints don't all move in sync."""
    wps = [(0.0, dict(start))]
    t = 0.0
    for _ in range(max(1, cycles)):
        t += secs
        out = dict(start)
        for k, j in enumerate(joints):
            sign = -1.0 if (complex_mode and k % 2) else 1.0
            out[j] += sign * delta
        wps.append((t, out))
        t += secs
        wps.append((t, dict(start)))
    return wps


def run_scaled(node, wps):
    if not node.ac.wait_for_server(timeout_sec=3.0):
        node.get_logger().error(
            'scaled_joint_trajectory_controller action not available — activate it:\n'
            '  ros2 control switch_controllers '
            '--deactivate forward_position_controller '
            '--activate scaled_joint_trajectory_controller')
        return False
    traj = JointTrajectory()
    traj.joint_names = UR_JOINTS
    for t, pos in wps[1:]:                      # skip the t=0 "current" point
        pt = JointTrajectoryPoint()
        pt.positions = [pos[j] for j in UR_JOINTS]
        pt.time_from_start = make_duration(t)
        traj.points.append(pt)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = node.ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        node.get_logger().error('Goal rejected.'); return False
    print('Goal accepted, executing...')
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    print('Done. error_code =', res_fut.result().result.error_code)
    return True


def run_fpc(node, wps, rate, ease=True):
    if node.count_subscribers(FPC_TOPIC) == 0:
        node.get_logger().error(
            f'no subscriber on {FPC_TOPIC} — forward_position_controller is not '
            'active (or nothing is listening). Activate it:\n'
            '  ros2 control switch_controllers '
            '--deactivate scaled_joint_trajectory_controller '
            '--activate forward_position_controller')
        return False
    total = wps[-1][0]
    n = int(total * rate)
    print(f'Streaming {n} snapshots at {rate} Hz...')
    for i in range(n + 1):
        t = i / rate
        pos = wps[-1][1]
        for j in range(len(wps) - 1):
            t0, p0 = wps[j]; t1, p1 = wps[j + 1]
            if t0 <= t <= t1:
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                if ease:
                    # cosine ease-in-out: velocity starts and ends at 0 -> no start jerk
                    a = 0.5 - 0.5 * math.cos(math.pi * a)
                pos = {nm: p0[nm] + a * (p1[nm] - p0[nm]) for nm in UR_JOINTS}
                break
        msg = Float64MultiArray()
        msg.data = [pos[nm] for nm in UR_JOINTS]
        node.cmd_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / rate)
    print('Done.')
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--joints', default='shoulder_lift_joint',
                   help='comma-separated joints to move together, e.g. '
                        'shoulder_pan_joint,shoulder_lift_joint')
    p.add_argument('--deg', type=float, default=10.0, help='relative move size (deg)')
    p.add_argument('--secs', type=float, default=4.0, help='seconds per leg (out, back)')
    p.add_argument('--speed', type=float, default=None,
                   help='deg/s; if set, overrides --secs (secs = deg / speed). '
                        'e.g. --speed 8 to match /go_home')
    p.add_argument('--cycles', type=int, default=1, help='out-and-back cycles')
    p.add_argument('--complex', action='store_true',
                   help='alternate sign per joint (joints move opposite ways)')
    p.add_argument('--controller', choices=['scaled', 'fpc'], default='scaled')
    p.add_argument('--rate', type=float, default=30.0, help='stream rate for fpc mode (Hz)')
    p.add_argument('--raw', action='store_true',
                   help='fpc only: DISABLE easing (linear ramp, abrupt start/stop) '
                        '— to test whether jerk causes the noise')
    p.add_argument('--list', action='store_true', help='print current joints and exit')
    args = p.parse_args()

    joints = [j.strip() for j in args.joints.split(',') if j.strip()]
    bad = [j for j in joints if j not in UR_JOINTS]
    if bad:
        print(f'unknown joint(s): {bad}\nvalid: {UR_JOINTS}'); sys.exit(1)

    deg = max(-MAX_DEG, min(MAX_DEG, args.deg))
    secs = abs(deg) / args.speed if args.speed else args.secs
    secs = max(MIN_SECS, secs)
    delta = math.radians(deg)
    # eased peak step per joint (cosine ease peaks at pi/2 x the average velocity)
    per_tick_peak = abs(deg) / (secs * args.rate) * (math.pi / 2)

    rclpy.init()
    node = Tester()
    if not node.wait_for_state():
        node.get_logger().error('No /joint_states — is the driver up (ur10e_start.sh)?')
        rclpy.shutdown(); sys.exit(1)

    start = {j: float(node.pos[j]) for j in UR_JOINTS}
    print('\nCurrent joint positions (deg):')
    for j in UR_JOINTS:
        print(f'  {j:20s} {math.degrees(start[j]):7.2f}')
    if args.list:
        rclpy.shutdown(); return

    wps = build_waypoints(start, joints, delta, secs, args.cycles, args.complex)
    print(f'\nPLAN: {",".join(joints)}  {deg:+.1f} deg out and back, {secs:.1f}s/leg, '
          f'{args.cycles} cycle(s){" [complex/opposed]" if args.complex else ""}  '
          f'via {args.controller}'
          f'{" @ %g Hz" % args.rate if args.controller == "fpc" else ""}')
    if args.controller == 'fpc':
        warn = '  <-- big step, watch for protective stop' if per_tick_peak > 2.0 else ''
        print(f'  fpc peak step: {per_tick_peak:.2f} deg/tick (eased){warn}')
    print('Listen to the control box. E-stop in hand. Sending in 3 s... (Ctrl-C to abort)')
    try:
        for i in (3, 2, 1):
            print(f'  {i}...'); rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print('aborted'); rclpy.shutdown(); return

    ok = run_fpc(node, wps, args.rate, ease=not args.raw) if args.controller == 'fpc' \
        else run_scaled(node, wps)
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
