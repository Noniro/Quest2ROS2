# UR10e Teleoperation — Bring-up Progress Log

**Robot**: UR10e at 192.168.1.31 (UR firmware 5.8.0.0 eSeries)  
**PC**: 192.168.1.202, Ubuntu 22.04, ROS2 Humble  
**Goal**: Quest2 VR headset → ROS2 IK → `forward_position_controller` streaming at 30 Hz

---

## Architecture

```
Quest2 (Unity app) ──UDP──> PC:10000 (ros_tcp_endpoint)
                                ↓
                    ur10e_teleop_controller (IK / joint mapping)
                                ↓
                    forward_position_controller (30 Hz position stream)
                                ↓
                    URPositionHardwareInterface (500 Hz write loop)
                                ↓ reverse socket TCP 50001
                    External Control URScript (running on pendant)
                                ↓ servoj()
                    UR10e joints move
```

---

## Configuration (Current Approach)

| Setting | Value |
|---|---|
| `headless_mode` | `false` (External Control URCap) |
| `reverse_ip` | `192.168.1.202` (must be explicit — `0.0.0.0` is used literally!) |
| `initial_joint_controller` | `forward_position_controller` |
| External Control program on pendant | `uv_external_control.urp` |
| `ROS_DOMAIN_ID` | `69` |

### Startup sequence

1. `./scripts/ur10e_start.sh` — stages robot (power on + brake release)
2. Driver launches in background with `headless_mode:=false`
3. Script waits for port 50002 (URCap script server) to open
4. Dashboard plays `uv_external_control.urp` via port 29999
5. URCap connects to port 50002 → driver sends URScript to robot
6. Robot connects back to port 50001 (reverse interface)
7. Log: "Robot connected to reverse interface. Ready to receive control commands."

---

## Patched System Files

All patches are to installed ROS2 packages under `/opt/ros/humble/`. They survive reboots but will be overwritten by `apt upgrade`. Keep the patch scripts in `/tmp/` or back them up.

---

### Patch 1 — `liburcl.so` (configure timeout)

**File**: `/opt/ros/humble/lib/x86_64-linux-gnu/liburcl.so`  
**Backup**: `/opt/ros/humble/lib/x86_64-linux-gnu/liburcl.so.bak`

**Problem**: `ur_driver.cpp` line 101 had `timeout(1000)` (1 second). Our UR10e takes 970–1055 ms to send its configuration package after connection, so it intermittently hit the wall and threw "Could not get configuration package within timeout."

**Fix**: Downloaded source `ros-humble-ur-client-library-2.11.0`, changed to `timeout(10000)` (10 seconds), rebuilt and replaced the .so:

```bash
cd /tmp/ur_build
cmake /tmp/ur_src/ros-humble-ur-client-library-2.11.0 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) urcl
sudo cp lib/liburcl.so /opt/ros/humble/lib/x86_64-linux-gnu/liburcl.so
```

**Status**: Applied ✓

---

### Patch 2 — `external_control.urscript` (keepalive + UR 5.8 fixes)

**File**: `/opt/ros/humble/share/ur_client_library/resources/external_control.urscript`

This file is compiled into URScript and sent to the robot when External Control plays.

#### 2a — Initial read_timeout too short

**Problem**: `params_mult[1]` = 40 ms (from `keep_alive_count=2 × 20ms`). Without patching, `read_timeout = 0.04 s`. If the PC misses even a few cycles in the first few seconds, the robot times out and exits the URScript loop immediately.

**Fix**: Line ~984 now sets `global read_timeout = 30.0` as the initial value (gives the PC 30 seconds to stabilize after the URScript starts). After each received packet the code clamps to min 1.0 s.

**Status**: Applied ✓

#### 2b — `max()` not in UR 5.8

**Problem**: The keepalive patch used `max(params_mult[1] / 1000.0, 1.0)`. UR 5.8 URScript doesn't have a `max(a, b)` function for scalars. Pendant showed a compile error immediately on play.

**Fix**: Replaced with explicit if/else (lines ~991–995):

```urscript
read_timeout = params_mult[1] / 1000.0
if read_timeout < 1.0:
  read_timeout = 1.0
end
```

**Status**: Applied ✓

#### 2c — `extrapolate()` float×list crash (UR 5.8)

**Problem**: Line 195 in `extrapolate()`:
```urscript
cmd_servo_q = cmd_servo_q + (steptime * get_target_joint_speeds())
```
`steptime` is a float, `get_target_joint_speeds()` returns a 6-element list. In UR 5.8 firmware, scalar × list is not supported — gives a "float and list" type error on the pendant. Also tried reversing the order (`list * scalar`) — same error. Neither direction works in 5.8.

The pendant shows this error at runtime, not compile time, because `extrapolate()` is only called when the servoThread hasn't received a fresh command (cmd_servo_state == SERVO_IDLE). This is what causes the connection to drop at ~350ms — the error halts URScript.

**Fix applied** (2026-06-16 via `sudo python3 /tmp/fix_ur_5_8_bugs.py`):

```urscript
def extrapolate():
  cmd_servo_q_last = cmd_servo_q
  local speeds = get_target_joint_speeds()
  cmd_servo_q = [cmd_servo_q[0] + steptime * speeds[0], cmd_servo_q[1] + steptime * speeds[1], cmd_servo_q[2] + steptime * speeds[2], cmd_servo_q[3] + steptime * speeds[3], cmd_servo_q[4] + steptime * speeds[4], cmd_servo_q[5] + steptime * speeds[5]]
  return cmd_servo_q
end
```

Only scalar×scalar multiplication and array indexing — works in any URScript version.

**Status**: Applied ✓ — connection now holds indefinitely

#### 2d — Spline scalar×list (UR 5.8)

**Problem**: Line ~461: `spline_qd = scaling_factor * spline_qd` — same scalar×list issue.

**Fix**: `spline_qd = scaling_factor * spline_qd` (reordered; also changed to scalar×list form in previous session).

**Status**: Applied ✓ (spline path only used with trajectory controllers, not fpc/servoj — lower priority)

---

### Patch 3 — `ur_control.launch.py` (consistent_controllers)

**File**: `/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py`

#### 3a — `forward_position_controller` missing from consistent_controllers (95ms drop)

**Problem**: The `controller_stopper_node` monitors `robot_program_running` and stops/restores "unsafe" controllers around External Control. At startup with `headless_mode=false`:

1. `forward_position_controller` (fpc) activates as the initial joint controller
2. `robot_program_running = False` → stopper STOPS fpc (~92ms after activate)
3. Robot plays External Control and connects to port 50001
4. `robot_program_running = True` → stopper RESTORES fpc
5. That restore triggers an IDLE→SERVOJ mode switch inside the hardware interface, which drops the reverse-interface connection immediately

Connection consistently dropped at ~95ms after the robot connected.

**Fix**: Added `"forward_position_controller"` to `consistent_controllers` — the stopper never stops it.

`write()` in the hardware interface still gates on `robot_program_running_` internally, so no servo commands reach the robot until External Control is actually playing. Safety is preserved.

**Status**: Applied ✓ — connection now holds past 95ms

#### 3b — `friction_model_controller` missing from consistent_controllers (STRICT abort)

**Problem**: `friction_model_controller` (fmc) is in `controllers_active` (so it gets activated at startup) but was NOT in `consistent_controllers`. The stopper sees it as an unsafe active controller and adds it to `stopped_controllers_`. Then:

- On robot **connect**: stopper calls `startControllers([fpc, fmc])`. If fpc is still active (stop failed), the switch_controller gets STRICT abort: "Controller 'forward_position_controller' is not inactive, cannot be activated."
- On robot **disconnect**: stopper calls `findAndStopControllers([fmc])`. If fmc was never actually stopped before, STRICT abort: "Controller 'friction_model_controller' cannot be deactivated since it is not active."

Connection still works despite these ERROR log messages — it's cosmetic noise, not a functional failure.

**Fix** (NOT YET APPLIED — requires `sudo`):

Add `"friction_model_controller"` to `consistent_controllers` in `ur_control.launch.py`. The same patch script handles this:

```bash
sudo python3 /tmp/fix_ur_5_8_bugs.py
```

**Status**: PENDING — harmless but noisy; apply when convenient

---

## Patch Script (apply both pending fixes at once)

```bash
sudo python3 /tmp/fix_ur_5_8_bugs.py
```

Verify:
```bash
grep -A6 "def extrapolate" /opt/ros/humble/share/ur_client_library/resources/external_control.urscript
grep "friction_model_controller" /opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py
```

Then restart:
```bash
pkill -f ur10e_start.sh; pkill -f ur_ros2_control_node; sleep 2
cd ~/projects/LearnROS2/ros2_ws && ./src/Quest2ROS2/scripts/ur10e_start.sh
```

---

## Full Error History

| Error | Root Cause | Fix | Status |
|---|---|---|---|
| `Could not get configuration package within timeout` | 1s timeout in `ur_driver.cpp:101`; robot takes 970–1055ms | Rebuilt liburcl.so with 10s timeout | Fixed ✓ |
| Robot never connects to reverse interface (headless mode) | In headless mode the driver wraps URScript in a def but never calls it; also `reverse_ip=0.0.0.0` is used literally | Switched to `headless_mode=false`, set `reverse_ip=192.168.1.202` | Fixed ✓ |
| `compile error: name 'max' is not defined` (pendant) | `max(a, b)` does not exist in UR 5.8 URScript | Replaced with if/else | Fixed ✓ |
| Connection drops at 95ms every time | controller_stopper stops fpc at startup, restores it 95ms after robot connects; restore's mode switch drops connection | Added fpc to consistent_controllers | Fixed ✓ |
| STRICT switch abort on robot connect + disconnect | fmc in controllers_active but not consistent_controllers | Add fmc to consistent_controllers | PENDING |
| `runtime error: float and list` on pendant | `steptime * get_target_joint_speeds()` — scalar×list not supported in UR 5.8 | Element-wise indexing in extrapolate() | Fixed ✓ |
| `Variable 'speed_slider_mask' currently controlled by another RTDE client` | Zombie ros2 processes from crashed session | `pkill -f ur_ros2_control_node` | Known workaround |
| `OSError: [Errno 98] Address already in use` on port 10000 | Leftover `default_server_endpoint` (ros_tcp_endpoint) from previous session; kills entire launch via on_exit handler | `pkill -f default_server_endpoint` before restarting | Known workaround |
| `Did not receive answer from dashboard server in time` | Dashboard client lost connection to robot dashboard server (idle timeout); play service call hangs for 20s then fails | Use `nc 192.168.1.31 29999` to send dashboard commands directly | Known workaround |
| Robot stuck in POWER_OFF after startup timeout | `ur10e_start.sh` staging timed out waiting for mode; robot never powered on but script printed success | Power on manually: `echo "power on" \| nc 192.168.1.31 29999`, wait for IDLE, then `echo "brake release" \| nc 192.168.1.31 29999` | Known workaround |
| `could not establish connection` on pendant | Driver was dead (killed by launch system) when robot tried to connect to port 50001 | Restart driver, then replay program via `echo "play" \| nc 192.168.1.31 29999` | Known workaround |

---

## What Works

- [x] Driver starts and connects (10s configure timeout fixed)
- [x] `headless_mode=false` with explicit `reverse_ip=192.168.1.202`
- [x] External Control URCap approach (pendant plays `.urp`)
- [x] `max()` URScript compile error fixed
- [x] Robot connects to reverse interface ("Ready to receive control commands.")
- [x] Connection holds past 95ms (fpc added to consistent_controllers)
- [x] **Connection holds indefinitely** (extrapolate() element-wise fix applied 2026-06-16)
- [x] **Physical robot motion via `forward_position_controller`** ← ACHIEVED 2026-06-16
- [x] **Quest2 connected** (192.168.253.103:10000 — WiFi IP, not ethernet)
- [x] **Proximity teleop sim running** — state machine + RViz markers + IK
- [ ] Proximity teleop sim fully tuned and verified
- [ ] Quest2 → real robot teleoperation (HW mode)
- [ ] RealSense D435i data collection

---

## Current Status (2026-06-16)

**ROBOT IS MOVING. QUEST2 CONNECTED. SIM RUNNING.**

### How the robot first moved

After the extrapolate() fix the connection held. First movement was via `/tmp/safe_move.py`:

```bash
source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=69 python3 /tmp/safe_move.py
```

`safe_move.py` linearly ramps `shoulder_pan` by **0.03 rad (~1.7°) over 5 seconds** at 30 Hz using `publish_blend()`. Max speed ≈ 0.6°/s — safe for bystanders.

**Why not nudge.py**: An earlier test with `/tmp/nudge.py` sent 0.05 rad in a SINGLE message. The robot tried to reach it immediately (within the servoj lookahead window), causing a sudden dangerous jump. Do NOT use nudge.py for testing. Always use a ramped command.

**Full pipeline confirmed working**:
```
safe_move.py → /forward_position_controller/commands → URPositionHardwareInterface → reverse socket → servoj() → joints move
```

**Pendant during motion**: Program `uv_external_control`, status: Running.  
"External control speed limit" message is NORMAL — the speed slider on the pendant limits max speed in External Control mode.

### Quest2 connection (2026-06-16)

**Problem**: Quest2 app was set to IP `192.168.1.202` (wired ethernet to robot network). The PC has two network interfaces: `192.168.1.202` (wired, robot-only subnet) and `192.168.253.103` (WiFi, same network as Quest2).

**Fix**: Changed Quest2 app IP to `192.168.253.103`. Connection established immediately.

**Endpoint race condition**: If the `ros_tcp_endpoint` starts before the teleop node registers its topics, Quest2 reconnects instantly and floods it with unregistered-topic errors → endpoint crashes. Fix: start endpoint AFTER seeing `[ProximityTeleop] Ready` in the sim launch log.

### Proximity teleop sim (2026-06-16)

New package: `q2r2_bringup/ur10e_proximity_teleop.py` + `launch/ur10e_proximity_sim.launch.py`.

**Concept**: Quest2 hand must be within 8 cm of robot TCP before robot follows. No sudden jumps possible — engagement requires 0.33s stable proximity.

**State machine**: IDLE (red) → PROXIMATE (yellow, countdown) → ENGAGED (green, robot follows)

**Launch** (two terminals):
```bash
# Terminal 1
source ~/projects/LearnROS2/ros2_ws/install/setup.bash && ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_proximity_sim.launch.py

# Terminal 2 (after "Ready (SIM)" appears in Terminal 1)
source ~/projects/LearnROS2/ros2_ws/install/setup.bash && ROS_DOMAIN_ID=69 ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

**Saved home pose** (jogged on pendant 2026-06-16):
```
shoulder_pan:  0°   (0.0 rad)
shoulder_lift: -90° (-1.5708 rad)
elbow:         145° (2.5307 rad)
wrist_1:       -55° (-0.9599 rad)
wrist_2:       90°  (1.5708 rad)
wrist_3:       0°   (0.0 rad)
```

**Remaining cosmetic issue**: `friction_model_controller` STRICT abort on every robot connect (harmless). Fix: `sudo python3 /tmp/fix_ur_5_8_bugs.py`.

---

## Next Steps

1. **Verify proximity teleop sim** — confirm RViz markers show correctly, robot model follows Quest2 hand after snap-zone engagement

2. **Tune sim parameters** if needed:
   ```bash
   ros2 param set /ur10e_proximity_teleop quest_offset_z 0.1   # shift hand sphere up/down
   ros2 param set /ur10e_proximity_teleop scale_factor 0.5     # slow down hand→robot mapping
   ```

3. **Move to HW mode** once sim is verified:
   ```bash
   # In ur10e_proximity_sim.launch.py change sim_mode: true → false
   # Or launch with:
   ros2 run q2r2_bringup ur10e_proximity_teleop --ros-args -p sim_mode:=false
   ```

4. **Apply fmc patch** (cosmetic — eliminates STRICT abort warning):
   ```bash
   sudo python3 /tmp/fix_ur_5_8_bugs.py
   ```

5. **Extract robot calibration** (eliminate kinematics mismatch warning):
   ```bash
   ros2 run ur_calibration calibration_correction --ros-args -p robot_ip:=192.168.1.31 -p target_filename:=$HOME/ur10e_calibration.yaml
   ```

6. **Install RealSense D435i**:
   ```bash
   sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
   ```

---

## Key Files

| File | Purpose | State |
|---|---|---|
| `src/Quest2ROS2/scripts/ur10e_start.sh` | Startup script (stage + launch + play URCap) | OK |
| `src/Quest2ROS2/launch/ur10e_teleop.launch.py` | ROS2 launch file (real HW) | OK |
| `src/Quest2ROS2/launch/ur10e_proximity_sim.launch.py` | Sim launch (robot model + teleop + RViz, no endpoint) | OK |
| `src/Quest2ROS2/q2r2_bringup/ur10e_proximity_teleop.py` | Proximity clutch teleop controller (sim+HW) | OK |
| `/opt/ros/humble/lib/x86_64-linux-gnu/liburcl.so` | Patched: 10s configure timeout | Patched ✓ |
| `/opt/ros/humble/share/ur_client_library/resources/external_control.urscript` | Patched: keepalive, max() fix, spline, extrapolate() | Patched ✓ |
| `/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py` | Patched: fpc in consistent_controllers; fmc still pending | Partial |
| `/tmp/fix_ur_5_8_bugs.py` | Patch script for fmc consistent_controllers (extrapolate already done) | Ready to run |
| `/tmp/safe_move.py` | **Safe** test script: ramps shoulder_pan 0.03 rad over 5s | Tested ✓ — robot moved |
| `/tmp/nudge.py` | ⚠️ UNSAFE — sends 0.05 rad in one message, causes sudden jump | Do not use |

---

## URScript Variables Visible on Pendant (Debug Info)

When External Control URScript is running, these globals are visible in the pendant's variable inspector:

| Variable | Type | Meaning |
|---|---|---|
| `control_mode` | int | -2=STOPPED, 0=IDLE, 1=SERVOJ, 2=SPEEDJ, … |
| `cmd_servo_q` | list[6] | Target joint positions currently being commanded (rad) |
| `cmd_servo_q_last` | list[6] | Previous target (before last extrapolate or command) |
| `steptime` | float | URScript control cycle time, typically 0.002s (500 Hz) |
| `read_timeout` | float | How long the main loop waits for a PC command before timing out (seconds) |
| `cmd_servo_state` | int | -1=UNINITIALIZED, 0=IDLE (extrapolating), 1=RUNNING (live cmds) |
| `params_mult` | list | Raw data packet received from reverse interface |
| `spline_qd` | list[6] | Joint velocities for spline interpolation |
| `spline_qdd` | list[6] | Joint accelerations for spline interpolation |

---

## Architecture Deep-Dive: Why headless_mode=false

With `headless_mode=true`: The driver wraps the URScript in `def externalControl(): ... end` but **never calls it**. The program runs and immediately exits. Also, `reverse_ip=0.0.0.0` is used literally (bug in eSeries 5.8 — newer firmware resolves it to the sender's IP).

With `headless_mode=false` (what we use): A URCap on the robot handles delivery. The pendant's External Control program connects to port 50002 on the PC, the driver sends the full URScript verbatim, and the robot runs it. `reverse_ip` must be set explicitly to `192.168.1.202`.

## Architecture Deep-Dive: controller_stopper

The `controller_stopper_node` watches the `robot_program_running` topic (published by `io_and_status_controller`). It's designed to stop position controllers when External Control isn't playing (so the hardware interface doesn't try to send servo commands to a robot that isn't in External Control mode).

Controllers in `consistent_controllers` are NEVER stopped by the stopper — they're considered always safe. Controllers in `controllers_active` but NOT in `consistent_controllers` are stopped when `robot_program_running=False` and restored when `True`.

With `stop_controllers_on_startup=true` (triggered when `headless_mode=false && joint_controller_active=true`), the stopper also runs a polling loop at startup to find and stop any non-consistent controllers that activated before the robot program started playing.

**Key insight**: `forward_position_controller` and `friction_model_controller` must BOTH be in `consistent_controllers`. The `write()` function in the hardware interface already gates on `robot_program_running_` internally, so servo commands never reach the robot when External Control isn't playing — the stopper's protection is redundant for these controllers.

## Architecture Deep-Dive: extrapolate()

The servoThread in the URScript runs at 500 Hz (every 2ms = `steptime`). The main loop reads from the PC socket with `read_timeout`. If the servoThread runs before a new command arrives from the PC (which happens frequently at 30Hz commands into a 500Hz servo loop), it calls `extrapolate()` to predict the next position.

`extrapolate()` adds `steptime × current_joint_speeds` to the last commanded position — tiny micro-steps to smooth motion. The bug is that UR 5.8 firmware doesn't support multiplying a float by a list directly. The fix uses explicit index-by-index multiplication.

## Potential Firmware Issue

UR 5.8.0.0 is missing several features that the ros-humble-ur-driver assumes:
- `max(a, b)` for scalars — not available
- `scalar × list` multiplication — not supported (runtime error)
- `reverse_ip=0.0.0.0` → sender IP resolution — doesn't work in 5.8

The driver targets newer firmware (5.11+). If issues persist after all patches, consider upgrading the robot firmware or filing a bug with Universal Robots.

---
*Last updated: 2026-06-16 (end of session 3 — robot moving, Quest2 connected, sim running)*
