# Quest 2 → Yaskawa HC10DTP — Native ROS 2 VR Teleoperation

Real-time teleoperation of a **Yaskawa HC10DTP** 6-axis collaborative robot using **Meta Quest 2** VR controllers, running **100% natively on ROS 2 Humble** (Ubuntu 22.04). No ROS 1 bridge, no Docker — the host PC talks directly to the robot's YRC1000micro cabinet via [MotoROS2](https://github.com/Yaskawa-Global/motoros2).

The system streams hand poses at 30 Hz, solves IK locally in ~7 ms with `ikpy`, and is gated by a clutch (deadman) plus two independent software safety clamps.

> Status: **simulation pipeline fully verified end-to-end** (2026-06-10). Hardware bring-up pending — see [Running on the real robot](#3-running-on-the-real-robot).
> Implementation plan & verification log: [HC10DTP_TELEOP_PLAN.md](./HC10DTP_TELEOP_PLAN.md).

---

## Architecture

```mermaid
flowchart TD
    subgraph VR ["Quest 2 Headset (Unity / quest2ros app)"]
        IMU["Controller tracking<br/>(cameras + IMU)"] --> OVR["Oculus OVR SDK"]
        OVR --> Q2R["quest2ros Unity publisher"]
    end

    subgraph Host ["Host PC — ROS 2 Humble, ROS_DOMAIN_ID=69"]
        EP["ros_tcp_endpoint<br/>(TCP server :10000)"]
        CTRL["hc10dtp_teleop_controller<br/>(30 Hz control loop)"]
        IK["ikpy chain solver<br/>(~7 ms per solve)"]
        RSP["robot_state_publisher<br/>(URDF → TF)"]
        RVIZ["RViz2"]

        EP -- "/q2r_right_hand_pose<br/>(PoseStamped)" --> CTRL
        EP -- "/q2r_right_hand_inputs<br/>(OVR2ROSInputs)" --> CTRL
        CTRL <-- "FK / IK queries" --> IK
        CTRL -- "/joint_states (sim mode)" --> RSP
        RSP -- "/tf" --> RVIZ
    end

    subgraph Cabinet ["YRC1000micro cabinet (ROS_DOMAIN_ID=69)"]
        MR2["MotoROS2 driver"]
        ROBOT["HC10DTP joints 1–6"]
        MR2 --> ROBOT
    end

    Q2R -- "WiFi TCP :10000" --> EP
    CTRL -- "trajectory commands (HW mode)" --> MR2
    MR2 -- "/joint_states (HW mode, best-effort QoS)" --> CTRL
```

Two mutually exclusive output paths, selected by the `sim_mode` parameter:

| Mode | Robot state source | Command sink | `/joint_states` publisher |
|---|---|---|---|
| `sim_mode:=true` | own integrated state | `/joint_states` → RViz | the teleop controller |
| `sim_mode:=false` | real robot via MotoROS2 | trajectory interface → cabinet | **MotoROS2 only** (controller never publishes it) |

---

## The Nodes

### 1. `ros_tcp_endpoint` (Unity bridge)
TCP server on port `10000`. The Quest 2 headset (running the quest2ros Unity app, configured with this PC's IP) connects over WiFi and its messages are re-published as native ROS 2 topics:
- `/q2r_right_hand_pose` — `geometry_msgs/PoseStamped`, controller pose in Unity frame (Y-up, Z-forward)
- `/q2r_right_hand_inputs` — `quest2ros/OVR2ROSInputs`, buttons / triggers / thumbstick

### 2. `hc10dtp_teleop_controller` (the brain — `q2r2_bringup/hc10dtp_teleop_controller.py`)
A single 30 Hz control-loop node. Per cycle:

1. **Clutch gate** — teleop runs only while enabled. The lower button (A) **toggles** enable/disable. Every engage re-anchors both the VR pose and the robot pose, so the robot never jumps to "catch up" with where your hand wandered while disengaged.
2. **Anchoring** — on engage, stores the current VR pose and the robot's end-effector pose. The robot pose comes from **the controller's own ikpy forward kinematics** (never TF), so a foreign `robot_state_publisher` or second robot on the network cannot corrupt it.
3. **Coordinate mapping** — relative VR displacement → robot base frame (`robot X ← −VR Z`, `robot Y ← −VR X`, `robot Z ← VR Y`), scaled by `scale_factor`. Relative VR rotation is conjugated into the robot frame by the fixed mapping matrix `R_vr_to_robot` and applied to the anchored orientation (full math in [HC10DTP_TELEOP_PLAN.md](./HC10DTP_TELEOP_PLAN.md)).
4. **IK** — `ikpy` solves position + full orientation against the chain parsed live from `hc10dtp_b00.xacro` (`motoman_hc10_support`), **seeded with the current joint state** for temporal consistency (prevents configuration flips).
5. **Safety clamps** — see [Safety](#safety-systems).
6. **Output** — `/joint_states` (sim) or a `JointTrajectory` point with 40 ms `time_from_start` (hardware).

In hardware mode it additionally **mirrors the real robot**: it subscribes to `/joint_states` (sensor-data QoS, matching MotoROS2's best-effort publisher) and adopts the real joint values whenever the clutch is disengaged — teleop always resumes from where the robot actually is.

**Parameters**

| Parameter | Default | Meaning |
|---|---|---|
| `sim_mode` | `true` | RViz-only vs real-robot output |
| `scale_factor` | `1.2` | hand-motion amplification |
| `publish_rate` | `30.0` | control loop Hz |
| `max_joint_delta` | `0.15` | max rad per joint per cycle before a solution is rejected |
| `x_min/x_max` | `0.15 / 0.95` | workspace box X (m) |
| `y_min/y_max` | `-0.75 / 0.75` | workspace box Y (m) |
| `z_min/z_max` | `-0.15 / 1.25` | workspace box Z (m) |
| `joint_states_topic` | `/joint_states` | retarget to a namespaced robot, e.g. `/yuval_hc10/joint_states` |
| `trajectory_topic` | `/joint_trajectory_controller/joint_trajectory` | command sink (will be adapted to the cabinet's actual MotoROS2 interface during bring-up) |

### 3. `robot_state_publisher`
Loads the HC10DTP URDF (processed from `hc10dtp_b00.xacro`) and broadcasts TF from `/joint_states`. Drives the RViz model.

### 4. `rviz2`
Visualization. Launched with `QT_QPA_PLATFORM=xcb` because RViz2 freezes/black-screens under GNOME Wayland (do **not** set `LIBGL_ALWAYS_SOFTWARE=1` on the host — that kills hardware OpenGL).

### 5. MotoROS2 (runs on the cabinet, not the PC)
Yaskawa's native ROS 2 driver, executing as a MotoPlus task on the YRC1000micro. Publishes `/joint_states` (best-effort) and `/robot_status`, and exposes motion interfaces (`follow_joint_trajectory` action / point-queue services). Its DDS settings live in `motoros2_config.yaml` **on the controller**.

---

## Safety Systems

| Layer | Mechanism | Effect |
|---|---|---|
| Clutch (deadman) | A-button toggle, re-anchors on every engage | release = robot freezes instantly; engage never jumps |
| Joint-jump clamp | any joint moving > `0.15 rad` in one 33 ms cycle | IK solution rejected, robot holds position |
| Workspace box | Cartesian target clipped to configured X/Y/Z box | can't be driven into the table / out of reach |
| State mirroring (HW) | adopts real `/joint_states` while clutch is off | commands always continue from the robot's true pose |
| HW preflight | [`scripts/hw_preflight.sh`](./scripts/hw_preflight.sh) | refuses to run with 2+ robots visible; audits the DDS domain before any motion |

Verified in sim (see `test/`): exact IK round-trip; 8 cm hand raise × 1.2 scale → tool0 +9.6 cm exactly; **zero** joint drift under violent VR jumps with the clutch off.

---

## DDS Domain Policy (two HC10s in one lab!)

A coworker operates a **second HC10 on the same network**. ROS 2 / DDS auto-discovers everything on the same domain ID, so separation is by domain:

| Who | `ROS_DOMAIN_ID` |
|---|---|
| **This project** — PC **and** our cabinet's MotoROS2 | **69** |
| Coworker | 0 (DDS default) or anything ≠ 69 — just not 69 |

Why a fixed unique ID (and not sharing 0 + being careful): domain 0 is where *everyone who forgets to export the variable* lands. On 69, his nodes can connect/disconnect whenever he wants and we are mutually invisible — no coordination needed beyond "69 is taken".

**Setup, once:**
1. PC: `echo 'export ROS_DOMAIN_ID=69' >> ~/.bashrc` (new terminals pick it up).
2. Cabinet: set `ros_domain_id: 69` in `motoros2_config.yaml` on the YRC1000micro (edit the YAML, reload via USB/pendant per the MotoROS2 docs, reboot the cabinet). **Without this step the PC and robot won't see each other.**
3. Tell the coworker 69 is ours.

`scripts/hw_preflight.sh` stays as the belt-and-suspenders check: if it ever reports two MotoROS2 nodes on domain 69, something is misconfigured — it exits and blocks teleop.

---

## How to Run Everything

### 0. Build (once per code change)
```bash
cd ~/projects/LearnROS2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select quest2ros q2r2_bringup ros_tcp_endpoint motoman_hc10_support motoman_resources
source install/setup.bash
```
Dependencies: ROS 2 Humble, `ikpy` (`pip install ikpy`), `xacro`.

### 1. Simulation (no robot, no headset needed)
```bash
export ROS_DOMAIN_ID=69
ros2 launch q2r2_bringup hc10dtp_sim.launch.py        # rviz:=false for headless
```
Starts `robot_state_publisher`, the teleop controller in `sim_mode:=true`, and RViz2. Drive it from a real headset (start the endpoint too — step 2) or run the automated fake-Quest test:
```bash
export ROS_DOMAIN_ID=69
python3 src/Quest2ROS2/test/test_teleop_e2e.py        # expects the sim launch to be up
python3 src/Quest2ROS2/test/test_hc10dtp_ik.py        # standalone IK round-trip check
```

### 2. Full stack with the headset (still simulated robot)
```bash
export ROS_DOMAIN_ID=69
ros2 launch q2r2_bringup hc10dtp_teleop.launch.py sim_mode:=true
```
Adds `ros_tcp_endpoint` on port `10000`. In the Quest 2 quest2ros app, set the ROS IP to this PC's WiFi address. Verify data: `ros2 topic hz /q2r_right_hand_pose`.

### 3. Running on the real robot
```bash
# 1) Power the cabinet, let MotoROS2 boot.
# 2) ALWAYS preflight first:
export ROS_DOMAIN_ID=69
./src/Quest2ROS2/scripts/hw_preflight.sh
#    - exactly ONE MotoROS2 node must be visible
#    - note which motion interface it exposes
# 3) Launch teleop in hardware mode:
ros2 launch q2r2_bringup hc10dtp_teleop.launch.py sim_mode:=false
```
First session: start with `scale_factor:=0.5`, hand near anchor, small slow moves.

> ⚠️ Bring-up note: MotoROS2 typically exposes a `follow_joint_trajectory` **action** and point-queue **services** rather than the trajectory topic this controller currently publishes. The preflight prints what the cabinet actually offers; wiring the controller to it is the one remaining integration step (plan, Step 7).

### Teleoperating
1. Hold the right controller comfortably, press **A** once → clutch **ENABLED** (logged), robot anchors to your hand.
2. Move your hand — the robot's tool follows (1.2× scaled). Rotate — the tool rotates.
3. Press **A** again → **DISABLED**, robot freezes; reposition your hand freely, press **A** to re-engage with zero jump.

---

## Repository Layout

```
Quest2ROS2/                        # ROS 2 package: q2r2_bringup
├── README.md                      # this file
├── HC10DTP_TELEOP_PLAN.md         # living plan: architecture math, checklist, verification log
├── HC10DTP_AGENT_PROMPT.md        # original build spec
├── q2r2_bringup/
│   ├── hc10dtp_teleop_controller.py   # the teleop node (see above)
│   └── ...                            # legacy SDA10-era nodes
├── launch/
│   ├── hc10dtp_sim.launch.py          # sim: RSP + controller + RViz
│   └── hc10dtp_teleop.launch.py       # full: + ros_tcp_endpoint, sim_mode arg
├── scripts/
│   └── hw_preflight.sh                # mandatory pre-hardware-session check
├── test/
│   ├── test_hc10dtp_ik.py             # ikpy chain FK/IK round-trip
│   └── test_teleop_e2e.py             # fake-Quest end-to-end sim test
└── Files_for_msg_pkg/                 # source of quest2ros msg definitions
```
Related workspace packages: `quest2ros` (the `OVR2ROSInputs`/haptics messages), `ros_tcp_endpoint`, `motoman_hc10_support` (URDF/meshes), `motoman_resources` (RViz config).

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| RViz2 black/frozen viewport on GNOME | Wayland/Qt conflict — launch files already force `QT_QPA_PLATFORM=xcb`. Never set `LIBGL_ALWAYS_SOFTWARE=1` on the host. |
| `ros2 topic list` shows ghosts of dead nodes | the ROS 2 daemon caches discovery: `ros2 daemon stop`, retry |
| Robot/joints jump around in RViz, tests flap | duplicate node instances fighting (e.g. two controllers publishing `/joint_states`): `pgrep -af hc10dtp` and kill strays |
| `[SAFETY] Joint delta command rejected` spam | anchor inconsistent with state, or hand moved too fast — re-toggle the clutch to re-anchor |
| PC sees nothing from the cabinet | domain mismatch — PC and `motoros2_config.yaml` must both say 69 |
| No `/q2r_*` topics | headset and PC not on the same WiFi / wrong ROS IP in the Quest app / endpoint not running |

---

## Legacy

This repo previously targeted the dual-arm **Motoman SDA10** through a ROS 1 bridge + Docker (see [TELEOP_ARCHITECTURE.md](./TELEOP_ARCHITECTURE.md) and git history). That path was abandoned: the DX100 streaming interface proved intermittently unreliable (non-deterministic command drops, ~30% execution rate in controlled tests). The HC10DTP + MotoROS2 stack replaces all of it.
