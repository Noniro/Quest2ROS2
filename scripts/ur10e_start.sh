#!/usr/bin/env bash
# UR10e startup — headless_mode=false (External Control URCap)
#
# Sequence:
#   1. Stage robot (power on, brake release)
#   2. Start driver in background (headless_mode=false)
#   3. Wait for script server port 50002 to be ready
#   4. Play uv_external_control.urp via dashboard
#   5. URCap connects to port 50002, driver sends URScript, robot connects to 50001
#
# Usage: ROS_DOMAIN_ID=69 ./scripts/ur10e_start.sh [robot_ip]

set -eo pipefail
ROBOT_IP="${1:-192.168.1.31}"
DOMAIN="${ROS_DOMAIN_ID:-69}"

source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GRN}[OK]${NC}  $*"; }
info() { echo -e "${YLW}[  ]${NC}  $*"; }
fail() { echo -e "${RED}[!!]${NC}  $*"; }

info "Step 1/3 — Verifying robot is reachable at ${ROBOT_IP}..."
if ! ping -c 1 -W 2 "${ROBOT_IP}" &>/dev/null; then
    fail "Cannot ping ${ROBOT_IP}. Check ethernet cable and robot power."
    exit 1
fi
ok "Robot reachable"

info "Step 2/3 — Preparing robot (power on → brake release)..."
python3 - "${ROBOT_IP}" <<'PYEOF'
import socket, time, sys

ip = sys.argv[1]

def connect(ip):
    s = socket.socket()
    s.settimeout(5)
    s.connect((ip, 29999))
    welcome = s.recv(1024).decode().strip()
    print(f"[dashboard] Connected: {welcome}")
    return s

def send(sock, command):
    sock.sendall((command + '\n').encode())
    time.sleep(0.3)
    try:
        resp = sock.recv(4096).decode().strip()
    except socket.timeout:
        resp = '(no response)'
    print(f"[dashboard] {command!r} -> {resp!r}")
    return resp

print(f"[dashboard] Connecting to {ip}:29999 ...")
s = connect(ip)

# The UR dashboard grants Remote Control privileges at connect-time.
# If session was opened in Local mode, reconnect to get Remote privileges.
test = send(s, 'robotmode')
if 'not allowed' in test or 'safety' in test.lower():
    print("[dashboard] Reconnecting to get Remote privileges...")
    s.close(); time.sleep(0.5); s = connect(ip)

def wait_for_mode(sock, target_modes, timeout=30):
    print(f"[dashboard] Waiting for robot mode in {target_modes} ...")
    for i in range(timeout):
        sock.sendall(b'robotmode\n')
        time.sleep(1)
        try:
            resp = sock.recv(4096).decode().strip()
        except socket.timeout:
            resp = 'Robotmode: UNKNOWN'
        print(f"[dashboard] {resp} ({i+1}s)")
        if any(m in resp for m in target_modes):
            return True
    print("[dashboard] WARNING: timed out waiting for mode")
    return False

send(s, 'stop')
time.sleep(1)

s.sendall(b'robotmode\n'); time.sleep(0.5)
try: mode_resp = s.recv(4096).decode().strip()
except socket.timeout: mode_resp = ''
print(f"[dashboard] After stop: {mode_resp}")

if 'RUNNING' in mode_resp:
    print("[dashboard] Stuck in RUNNING — power cycling...")
    send(s, 'power off')
    wait_for_mode(s, ['POWER_OFF'], timeout=20)
    time.sleep(1)

send(s, 'power on')
wait_for_mode(s, ['IDLE'])
send(s, 'brake release')
wait_for_mode(s, ['IDLE', 'RUNNING'])
time.sleep(2)
s.close()
print("[dashboard] Done — robot staged in IDLE")
PYEOF

ok "Robot powered on, brakes released"

info "Step 3/3 — Starting driver (headless_mode=false) and playing External Control..."
export ROS_DOMAIN_ID=${DOMAIN}
ros2 launch q2r2_bringup ur10e_teleop.launch.py \
    robot_ip:="${ROBOT_IP}" \
    headless_mode:=false &
LAUNCH_PID=$!

# Wait for the script server (port 50002) to open — driver is ready for URCap connection
info "Waiting for driver script server on port 50002..."
WAITED=0
for i in $(seq 1 80); do
    if ss -tlnp 2>/dev/null | grep -q ':50002 '; then
        ok "Script server ready (${i}00ms)"
        WAITED=$i
        break
    fi
    sleep 0.1
done

if ! ss -tlnp 2>/dev/null | grep -q ':50002 '; then
    fail "Script server never opened — driver may have crashed"
    wait $LAUNCH_PID
    exit 1
fi

# Play the External Control program — URCap will connect to port 50002
python3 - "${ROBOT_IP}" <<'PYEOF'
import socket, time, sys
ip = sys.argv[1]
s = socket.socket(); s.settimeout(5)
s.connect((ip, 29999))
print(s.recv(1024).decode().strip())

def send(sock, command):
    sock.sendall((command + '\n').encode()); time.sleep(0.3)
    try: resp = sock.recv(4096).decode().strip()
    except socket.timeout: resp = '(no response)'
    print(f"[dashboard] {command!r} -> {resp!r}"); return resp

send(s, 'load /programs/uv_external_control.urp')
time.sleep(0.5)
send(s, 'play')
s.close()
print("[dashboard] Program playing — URCap connecting to driver")
PYEOF

ok "External Control program playing"
wait $LAUNCH_PID
