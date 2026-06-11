#!/usr/bin/env bash
# HC10DTP hardware preflight — run BEFORE any real-robot teleop session.
#
# Checks, in order:
#   1. Which DDS domain we are on.
#   2. Which MotoROS2 / robot nodes are visible — and warns loudly if more
#      than one robot is discoverable (a coworker's HC10 shares this lab
#      network; commanding or mirroring the wrong robot must be impossible).
#   3. Which motion interface the cabinet actually exposes (topic vs
#      follow_joint_trajectory action vs point-queue services).
#
# Usage:  ROS_DOMAIN_ID=69 ./hw_preflight.sh
#
# Project policy: OUR domain is 69 (PC + cabinet motoros2_config.yaml).
# The coworker's HC10 lives on domain 0/default. If this script finds his
# robot on 69, the cabinet configs are wrong — fix before any motion.

set -u

echo "=============================================="
echo " HC10DTP HW PREFLIGHT  (domain ${ROS_DOMAIN_ID:-0})"
echo "=============================================="

# Fresh discovery — the ros2 daemon caches stale graph info aggressively.
ros2 daemon stop >/dev/null 2>&1
sleep 1

echo
echo "--- Nodes visible on this domain ---"
NODES=$(timeout 20 ros2 node list 2>/dev/null)
echo "${NODES:-<none>}"

ROBOT_NODES=$(echo "$NODES" | grep -ciE "motoman|motoros" || true)
echo
if [ "$ROBOT_NODES" -eq 0 ]; then
    echo "RESULT: no MotoROS2 node visible. Cabinet is off, not booted, or on a"
    echo "        different ROS_DOMAIN_ID. Ours should be 69 (PC and cabinet"
    echo "        motoros2_config.yaml). Re-run with ROS_DOMAIN_ID=<n> to scan."
    exit 1
elif [ "$ROBOT_NODES" -gt 1 ]; then
    echo "!! STOP: MULTIPLE MotoROS2 nodes visible — the coworker's HC10 is on"
    echo "!! this domain too. Do NOT launch teleop here. Either move our cabinet"
    echo "!! to its own ros_domain_id (motoros2_config.yaml) or namespace it,"
    echo "!! then re-run this preflight."
    exit 2
fi
echo "OK: exactly one MotoROS2 node visible."

# Foreign control stacks (move_group etc.) sharing the domain are a red flag
# even with one robot: they may also be commanding it.
FOREIGN=$(echo "$NODES" | grep -iE "move_group|motion_service|lucid|lake" || true)
if [ -n "$FOREIGN" ]; then
    echo
    echo "!! WARNING: other control stacks visible on this domain:"
    echo "$FOREIGN"
    echo "!! Confirm with their owner before streaming motion."
fi

echo
echo "--- /joint_states publishers (must be exactly the robot) ---"
timeout 15 ros2 topic info /joint_states --verbose 2>/dev/null \
    | grep -B1 "Endpoint type: PUBLISHER" | grep "Node name" || echo "<none>"

echo
echo "--- Motion interfaces exposed by the cabinet ---"
echo "[actions]"
timeout 15 ros2 action list 2>/dev/null | grep -iE "trajectory|motion" || echo "  <no trajectory action>"
echo "[trajectory topics]"
timeout 15 ros2 topic list 2>/dev/null | grep -iE "trajectory" || echo "  <no trajectory topic>"
echo "[MotoROS2 services]"
timeout 15 ros2 service list 2>/dev/null \
    | grep -iE "queue_traj_point|start_traj_mode|start_point_queue_mode|stop_traj_mode|reset_error|select_motion_tool" \
    || echo "  <no MotoROS2 motion services found>"

echo
echo "--- Robot status ---"
timeout 10 ros2 topic echo /robot_status --once 2>/dev/null || echo "<no /robot_status>"

echo
echo "Preflight done. For streaming teleop prefer point-queue mode"
echo "(start_point_queue_mode + queue_traj_point) if listed above; otherwise"
echo "wire the controller to the follow_joint_trajectory action."
