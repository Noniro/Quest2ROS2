# Master Agent Prompt: Recreating the Yaskawa HC10DTP ROS 2 Teleoperation Suite

This document is a comprehensive, production-grade system prompt designed for an AI coding agent. When fed to a new agent, it provides all system architectures, code designs, spatial transformations, build sequences, and troubleshooting steps needed to reconstruct or extend the Yaskawa HC10DTP Quest 2 teleoperation workspace from scratch in native ROS 2 Humble.

---

## Workspace Setup & Sourcing Context

### 1. Target Directory
You MUST execute all builds, package creations, and node executions from the **ROS 2 workspace root**:
```
/home/yuval/projects/LearnROS2/ros2_ws
```
**Why?** Sourcing local workspace packages and executing `colcon build` requires the context of the workspace root. Do not execute workspace-level builds from subdirectories.

### 2. Workspace Structure
Your workspace has the following structure under `src/`:
* `src/Quest2ROS2/`: The primary package containing teleop scripts and launchers.
  * Package name: `q2r2_bringup` (defined in `setup.py` / `package.xml`).
  * Launcher scripts in `launch/`.
  * Controller scripts in `q2r2_bringup/`.
* `src/quest2ros/`: Custom package defining Quest 2 message specifications.
  * Custom inputs message: `quest2ros/msg/OVR2ROSInputs`
* `src/ros_tcp_communication/`: Native ROS 2 package containing the `ros_tcp_endpoint` node.
* `src/motoman_ros2_support_packages/`: Standard robot description packages, including:
  * `motoman_hc10_support`: Contains `urdf/hc10dtp_b00.xacro`
  * `motoman_resources`: Contains `rviz/view_robot.rviz`

### 3. Build & Compile Sequence (CRITICAL)
Custom message definitions in `quest2ros` must compile before `q2r2_bringup` is built, or Python imports of custom messages will fail. Execute these commands exactly:
```bash
# 1. Source ROS 2 system installation
source /opt/ros/humble/setup.bash

# 2. Build custom messages package first
colcon build --packages-select quest2ros

# 3. Source the generated message interfaces
source install/setup.bash

# 4. Build remaining packages
colcon build

# 5. Final source for workspace execution
source install/setup.bash
```

---

## System Topology & Communication Flow

The system runs entirely natively on the host PC (Ubuntu 22.04, ROS 2 Humble). It eliminates legacy ROS 1 components, Docker containers, and UDP socket wrappers.

```
[Quest 2 Headset (Unity / Quest2ROS)]
          │ (WiFi TCP Port 10000)
          ▼
[ros_tcp_endpoint (Host Node)] ──> /q2r_right_hand_pose (PoseStamped)
          │                    ──> /q2r_right_hand_inputs (OVR2ROSInputs)
          ▼
[hc10dtp_teleop_controller (Host Node)] <──> [Local ikpy Solver (~2ms)]
          │
          ├──(sim_mode := true) ──> /joint_states ──> [robot_state_publisher] ──> [RViz2]
          └──(sim_mode := false)──> /joint_trajectory_controller/joint_trajectory ──> [MotoROS2 Cabinet]
```

---

## Coordinate Transformations & Spatial Math

### 1. Position Mapping (Unity VR to Robot Base)
Quest 2 controller coordinates are represented in Unity's coordinate frame (Y-up, X-right, Z-forward). Map their relative displacement offsets to the Yaskawa robot base link coordinates (Z-up, Y-left, X-forward):
* **Robot X (Forward)** $\leftarrow$ VR $-Z$ (Backward)
* **Robot Y (Left)** $\leftarrow$ VR $-X$ (Right/Left)
* **Robot Z (Up)** $\leftarrow$ VR $Y$ (Up/Down)

$$\vec{P}_{target\_robot} = \vec{P}_{anchor\_robot} + \text{Scale} \times \begin{bmatrix} -\Delta Z_{vr} \\ -\Delta X_{vr} \\ \Delta Y_{vr} \end{bmatrix}$$

* **Scale Factor**: Typically set to `1.2` to amplify hand motion, or parameterizable.

### 2. Rotation Matrix Mapping
To align the Quest controller's orientation with the robot gripper (`tool0`), use the mapping matrix $R_{vr\_to\_robot}$:

$$R_{vr\_to\_robot} = \begin{bmatrix} 
 0.0 & 0.0 & -1.0 \\ 
-1.0 & 0.0 &  0.0 \\ 
 0.0 & 1.0 &  0.0 
\end{bmatrix}$$

Compute the target orientation matrix ($R_{target}$) to feed into the IK solver:
1. Get current VR controller rotation matrix $R_{vr}$ and start anchor $R_{vr\_anchor}$.
2. Compute relative VR rotation matrix: $R_{vr\_rel} = R_{vr\_anchor}^{T} \cdot R_{vr}$.
3. Rotate relative change into robot frame: $R_{vr\_rel\_robot} = R_{vr\_to\_robot} \cdot R_{vr\_rel} \cdot R_{vr\_to\_robot}^{T}$.
4. Apply to start robot orientation anchor: $R_{target} = R_{robot\_anchor} \cdot R_{vr\_rel\_robot}$.

---

## Low-Latency Inverse Kinematics (IK) via `ikpy`

> [!IMPORTANT]
> Do **not** use MoveIt 2 / OMPL motion planning for streaming teleoperation. The configuration overhead is massive, and tree-search planning introduces **200ms–2000ms latency** which makes real-time hand-tracking unsafe. Use `ikpy` locally to solve IK in **~2ms** at **30Hz**.

### Implementation Steps:
1. Load `hc10dtp_b00.xacro` (from `motoman_hc10_support`), convert it to XML URDF via `xacro`, write it to a temporary file, and load it using `ikpy.chain.Chain.from_urdf_file`.
2. Define the chain elements:
   `["base_link", "joint_1", "link_1", "joint_2", "link_2", "joint_3", "link_3", "joint_4", "link_4", "joint_5", "link_5", "joint_6", "link_6", "joint_6-flange", "flange", "flange-tool0", "tool0"]`
3. Define the active joint mask to ensure only the 6 revolute joints are solved:
   `[False, True, True, True, True, True, True, False, False]`
4. Use temporal consistency: seed the `inverse_kinematics` solver using the robot's current joint positions as the `initial_position` to prevent configuration flips.

---

## Safety Clamps & Clutch Controls

Implement the following safety guards in the Python node:
1. **Joint Jump Clamp**: Compare the solved joint values with the last published joint values. If any joint jumps by more than **$0.15\text{ rad}$ (~$8.5^\circ$)** in a single $33\text{ ms}$ step, reject the solution and hold the previous state.
2. **Workspace Bounding Box**: Clip the target Cartesian positions to a safe bounding box:
   * $X \in [0.15, 0.95]\text{ m}$
   * $Y \in [-0.75, 0.75]\text{ m}$
   * $Z \in [-0.15, 1.25]\text{ m}$
3. **Clutch deadman switch**: Teleoperation is only enabled when the user presses/holds the controller's lower button (`msg.button_lower` from `/q2r_right_hand_inputs`). When pressed, it resets the VR and robot anchors to prevent motion jumps. Releasing it freezes control.

---

## RViz2 GUI Configuration & Graphics Fixes

> [!CAUTION]
> **Wayland/Gnome Graphics Conflict Resolution**:
> Running RViz2 under Gnome Wayland displays a solid background color (RGB 48,48,48) and freezes the UI because hardware OpenGL acceleration fails when Qt falls back to XWayland.
> 
> * **The Fix**: The launch file must force the Qt backend to use the X11 platform:
>   `rviz_env["QT_QPA_PLATFORM"] = "xcb"`
> * **The Trap**: Do **NOT** set `rviz_env["LIBGL_ALWAYS_SOFTWARE"] = "1"` on the host machine. While this is needed inside headless Docker containers, on the host laptop it disables hardware OpenGL acceleration entirely, causing a black viewport screen.

---

## Node Interface Requirements

### Subscriptions:
* `/q2r_right_hand_pose` (`geometry_msgs/msg/PoseStamped`)
* `/q2r_right_hand_inputs` (`quest2ros/msg/OVR2ROSInputs`)

### Publishers:
* `/joint_states` (`sensor_msgs/msg/JointState`) (Sim mode)
* `/joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) (HW mode)
  * Point waypoint lookahead duration: **40ms** (`time_from_start = Duration(sec=0, nanosec=40000000)`).

---

## Hardware Execution & Networking (MotoROS2)

When running on the real YRC1000micro controller:
1. Ensure the YRC1000micro cabinet runs **MotoROS2** (configured to accept standard `JointTrajectory` commands).
2. Configure host PC routing: ensure the controller's subnet is reachable.
3. If using multiple network interfaces (e.g. WiFi for Quest 2 headset, Ethernet cable for MotoROS2), ROS 2 DDS may fail to route discovery packets correctly.
   - Resolve this by declaring the interface in a FastDDS configuration file and setting:
     `export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profiles.xml`
4. Launch the hardware mode:
   ```bash
   ros2 launch q2r2_bringup hc10dtp_teleop.launch.py sim_mode:=false
   ```
