#!/usr/bin/env bash
# UR10e hardware preflight — run BEFORE launching ur10e_teleop.launch.py
# Usage: ROS_DOMAIN_ID=69 ./scripts/ur10e_hw_preflight.sh <robot_ip>
#
# Exit codes:
#   0 — robot reachable, ROS2 driver visible, forward_position_controller available
#   1 — robot not reachable or driver not running
#   2 — unexpected nodes detected (possible collision with another session)

set -euo pipefail

ROBOT_IP="${1:-192.168.1.100}"
DOMAIN="${ROS_DOMAIN_ID:-69}"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GRN}[OK]${NC}  $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; }

echo "======================================================"
echo " UR10e Preflight  |  robot_ip=${ROBOT_IP}  domain=${DOMAIN}"
echo "======================================================"

# 1. Network reachability
echo -n "Pinging robot... "
if ping -c 2 -W 1 "${ROBOT_IP}" &>/dev/null; then
    ok "Robot at ${ROBOT_IP} is reachable"
else
    fail "Cannot reach ${ROBOT_IP} — check ethernet cable and robot power"
    exit 1
fi

# 2. Restart ROS daemon for a clean discovery slate
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start

# 3. Check for ur_ros2_control_node (published by ur_robot_driver)
echo -n "Waiting for ur_robot_driver nodes (up to 10 s)..."
for i in $(seq 1 10); do
    NODES=$(ros2 node list 2>/dev/null || true)
    if echo "${NODES}" | grep -q "ur_ros2_control_node\|controller_manager"; then
        echo ""
        ok "UR driver nodes found"
        break
    fi
    echo -n "."; sleep 1
done
if ! echo "${NODES:-}" | grep -q "controller_manager"; then
    echo ""
    fail "controller_manager not visible on domain ${DOMAIN}. Is ur_robot_driver running?"
    echo "  → Start it with:"
    echo "    ROS_DOMAIN_ID=${DOMAIN} ros2 launch q2r2_bringup ur10e_teleop.launch.py robot_ip:=${ROBOT_IP}"
    exit 1
fi

# 4. Check for duplicate/unexpected sessions
CTRL_NODES=$(echo "${NODES}" | grep "ur10e_teleop_controller" || true)
COUNT=$(echo "${CTRL_NODES}" | grep -c "ur10e_teleop_controller" || echo 0)
if [ "${COUNT}" -gt 1 ]; then
    fail "Multiple ur10e_teleop_controller nodes detected — kill stale sessions first"
    echo "${CTRL_NODES}"
    exit 2
fi

# 5. List available controllers
echo ""
echo "--- ros2_control controllers ---"
ros2 control list_controllers 2>/dev/null || warn "Could not list controllers (driver may still be starting)"

# 6. Check forward_position_controller state
FWD=$(ros2 control list_controllers 2>/dev/null | grep "forward_position_controller" || true)
if echo "${FWD}" | grep -q "active"; then
    ok "forward_position_controller is ACTIVE — ready for teleop"
elif echo "${FWD}" | grep -q "inactive"; then
    warn "forward_position_controller is inactive — launch file will activate it"
elif [ -z "${FWD}" ]; then
    warn "forward_position_controller not found — check ur_robot_driver config"
fi

# 7. Joint state check
echo ""
echo "--- Current joint positions ---"
timeout 3 ros2 topic echo --once /joint_states 2>/dev/null \
    | grep -E "name|position" || warn "No /joint_states received"

echo ""
echo "======================================================"
echo " Preflight complete. If all checks passed:"
echo "   ROS_DOMAIN_ID=${DOMAIN} ros2 launch q2r2_bringup ur10e_teleop.launch.py \\"
echo "       robot_ip:=${ROBOT_IP}"
echo "======================================================"
