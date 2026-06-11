# Master System Prompt: Yaskawa HC10DTP Native ROS 2 Teleoperation Suite

You are an agentic coding assistant tasked with building a native **ROS 2 Humble** bimanual/single-arm teleoperation system for the Yaskawa HC10DTP 6-axis collaborative robot controlled by Meta Quest 2 VR controllers. 

This project must be implemented from scratch, incorporating the design patterns, math transformations, and graphics configurations developed for the legacy Motoman SDA10 robot, but completely eliminating the ROS 1 bridge and Docker container environments.

---

## 1. System Topology & Data Flow

The system runs natively on the host PC (Ubuntu 22.04, ROS 2 Humble) and communicates with the real Yaskawa YRC1000micro controller via **MotoROS2**.

```
[Quest 2 Headset (Unity)] 
         │ (WiFi TCP Port 10000)
         ▼
[ros_tcp_endpoint (Host Node)] ──> /q2r_right_hand_pose
         │                         /q2r_right_hand_inputs
         ▼
[hc10dtp_teleop_controller (Host Node)] <──> [Local ikpy Solver (~2ms)]
         │
         ├──(sim_mode := true) ──> /joint_states ──> [robot_state_publisher] ──> [RViz2]
         └──(sim_mode := false)──> /joint_trajectory_controller/joint_trajectory ──> [MotoROS2 Cabinet]
```

---

## 2. Codebase Structure & Workspace Layout

Organize the workspace cleanly inside the ROS 2 package directory:
* **Package Directory**: `ros2_ws/src/Quest2ROS2/`
* **Python Executables Directory**: `ros2_ws/src/Quest2ROS2/q2r2_bringup/`
* **Launch Files Directory**: `ros2_ws/src/Quest2ROS2/launch/`

### File Manifest:
1. `q2r2_bringup/hc10dtp_teleop_controller.py`: Main control node subscribing to VR inputs, mapping coordinates, computing direct IK via `ikpy`, and publishing outputs.
2. `launch/hc10dtp_sim.launch.py`: Starts `robot_state_publisher` (loading `hc10dtp_b00.xacro`), starts the teleop controller in `sim_mode:=true`, and opens RViz2.
3. `launch/hc10dtp_teleop.launch.py`: Starts everything above, sets `sim_mode:=false`, and additionally launches `ros_tcp_endpoint`.

---

## 3. Coordinate Mappings & Rotation Mathematics

### Position Mapping
Quest 2 controller coordinates are represented in Unity's coordinate frame (Y-up, X-right, Z-forward). Map their relative displacement offsets to the Yaskawa robot base link coordinates (Z-up, Y-left, X-forward):
* **Robot X (Forward)** $\leftarrow$ VR $-Z$ (Backward)
* **Robot Y (Left)** $\leftarrow$ VR $-X$ (Right/Left)
* **Robot Z (Up)** $\leftarrow$ VR $Y$ (Up/Down)

$$\vec{P}_{target\_robot} = \vec{P}_{anchor\_robot} + \text{Scale} \times \begin{bmatrix} -\Delta Z_{vr} \\ -\Delta X_{vr} \\ \Delta Y_{vr} \end{bmatrix}$$

### Rotation Matrix Mapping
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

## 4. Local Low-Latency IK solving via `ikpy`

> [!IMPORTANT]
> Do **not** use MoveIt 2 / OMPL motion planning for streaming teleoperation. The configuration overhead is massive, and tree-search planning introduces a **200ms–2000ms latency** which makes real-time hand-tracking unsafe. Use `ikpy` locally to solve IK in **~2ms** at **30Hz**.

### Implementation Steps:
1. Load `hc10dtp_b00.xacro` (from `motoman_hc10_support`), convert it to XML URDF via `xacro`, write it to a temporary file, and load it using `ikpy.chain.Chain.from_urdf_file`.
2. Define the chain elements:
   `["base_link", "joint_1", "link_1", "joint_2", "link_2", "joint_3", "link_3", "joint_4", "link_4", "joint_5", "link_5", "joint_6", "link_6", "joint_6-flange", "flange", "flange-tool0", "tool0"]`
3. Define the active joint mask to ensure only the 6 revolute joints are solved:
   `[False, True, True, True, True, True, True, False, False]`
4. Use temporal consistency: seed the `inverse_kinematics` solver using the robot's current joint positions as the `initial_position` to prevent configuration flips.

---

## 5. Software Safety Clamps & Clutch Controls

Implement the following safety guards in the Python node:
1. **Joint Jump Clamp**: Compare the solved joint values with the last published joint values. If any joint jumps by more than **$0.15\text{ rad}$ (~$8.5^\circ$)** in a single $33\text{ ms}$ step, reject the solution and hold the previous state.
2. **Workspace Bounding Box**: Clip the target Cartesian positions to a safe bounding box:
   * $X \in [0.15, 0.95]\text{ m}$
   * $Y \in [-0.75, 0.75]\text{ m}$
   * $Z \in [-0.15, 1.25]\text{ m}$
3. **Clutch deadman switch**: Teleoperation is only enabled when the user presses/holds the controller's lower button (`msg.button_lower`). When pressed, it resets the VR and robot anchors to prevent motion jumps.

---

## 6. RViz2 GUI Configuration & Graphics Fixes

> [!CAUTION]
> **Wayland/Gnome Graphics Conflict Resolution**:
> Running RViz2 under Gnome Wayland displays a solid background color (RGB 48,48,48) and freezes the UI because hardware OpenGL acceleration fails when Qt falls back to XWayland.
> 
> * **The Fix**: The launch file must force the Qt backend to use the X11 platform:
>   `rviz_env["QT_QPA_PLATFORM"] = "xcb"`
> * **The Trap**: Do **NOT** set `rviz_env["LIBGL_ALWAYS_SOFTWARE"] = "1"` on the host machine. While this is needed inside headless Docker containers, on the host laptop it disables hardware OpenGL acceleration entirely, causing a black viewport screen.

---

## 7. Node Interface Requirements

### Subscriptions:
* `/q2r_right_hand_pose` (`geometry_msgs/msg/PoseStamped`)
* `/q2r_right_hand_inputs` (`quest2ros/msg/OVR2ROSInputs`)

### Publishers:
* `/joint_states` (`sensor_msgs/msg/JointState`) (Sim mode)
* `/joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) (HW mode)
  * Point waypoint lookahead duration: **40ms** (`time_from_start = Duration(sec=0, nanosec=40000000)`).
