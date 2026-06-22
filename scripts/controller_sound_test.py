#!/usr/bin/env python3
"""
controller_sound_test.py — diagnostic for the control-box noise during teleop.

Sends a SLOW, SMALL, RELATIVE point-to-point move on one joint through the
scaled_joint_trajectory_controller (smooth interpolation at servo rate — the SAME
execution path MoveIt / pendant point-to-point use), so you can listen and compare
it to forward_position_controller streaming (the teleop default).

SAFETY
  - Slow + small + relative moves only. E-stop in hand, pendant speed slider low.
  - Run only with the UR driver up (ur10e_start.sh) AND the teleop node IDLE
    (do NOT press A) or not running, so nothing else commands the arm.
  - The scaled_joint_trajectory_controller must be ACTIVE (see the switch command
    the script prints if the action server isn't found).

Examples
  # just print current joint positions, move nothing:
  python3 controller_sound_test.py --list

  # move shoulder_lift 10 deg out and back over 4 s each way, listen:
  python3 controller_sound_test.py --joint shoulder_lift_joint --deg 10 --secs 4

  # the noisy one — base, a bit bigger, 2 cycles:
  python3 controller_sound_test.py --joint shoulder_pan_joint --deg 15 --secs 5 --cycles 2
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'

# conservative safety caps
MAX_DEG = 30.0
MIN_SECS = 2.0


class Tester(Node):
    def __init__(self):
        super().__init__('controller_sound_test')
        self.pos = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.ac = ActionClient(self, FollowJointTrajectory, ACTION)

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


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--joint', default='shoulder_lift_joint', choices=UR_JOINTS)
    p.add_argument('--deg', type=float, default=10.0, help='relative move size (deg)')
    p.add_argument('--secs', type=float, default=4.0, help='seconds per leg (out, back)')
    p.add_argument('--cycles', type=int, default=1, help='number of out-and-back cycles')
    p.add_argument('--list', action='store_true', help='just print current joints and exit')
    args = p.parse_args()

    deg = max(-MAX_DEG, min(MAX_DEG, args.deg))
    secs = max(MIN_SECS, args.secs)
    delta = math.radians(deg)

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

    # build out-and-back trajectory on the chosen joint
    traj = JointTrajectory()
    traj.joint_names = UR_JOINTS
    t = 0.0
    for _ in range(max(1, args.cycles)):
        t += secs
        out = dict(start); out[args.joint] += delta
        pt = JointTrajectoryPoint()
        pt.positions = [out[j] for j in UR_JOINTS]
        pt.time_from_start = make_duration(t)
        traj.points.append(pt)

        t += secs
        pb = JointTrajectoryPoint()
        pb.positions = [start[j] for j in UR_JOINTS]
        pb.time_from_start = make_duration(t)
        traj.points.append(pb)

    print(f'\nPLAN: {args.joint}  {deg:+.1f} deg out and back, {secs:.1f}s per leg, '
          f'{args.cycles} cycle(s)  (controller: scaled_joint_trajectory_controller)')
    print('Listen to the control box. E-stop in hand. Sending in 3 s... (Ctrl-C to abort)')
    try:
        for i in (3, 2, 1):
            print(f'  {i}...'); rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print('aborted'); rclpy.shutdown(); return

    if not node.ac.wait_for_server(timeout_sec=3.0):
        node.get_logger().error(
            'scaled_joint_trajectory_controller action not available.\n'
            'Activate it first:\n'
            '  ros2 control switch_controllers '
            '--deactivate forward_position_controller '
            '--activate scaled_joint_trajectory_controller')
        rclpy.shutdown(); sys.exit(1)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = node.ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        node.get_logger().error('Goal rejected.'); rclpy.shutdown(); sys.exit(1)
    print('Goal accepted, executing...')
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    print('Done. error_code =', res_fut.result().result.error_code)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
