# Quest 2 Bimanual Robot Teleoperation System

A framework for using Meta Quest 2/3 VR controllers to teleoperate both arms of the Motoman SDA10 robot through ROS 2 and ROS-TCP communication.

For a detailed explanation of the coordinate transformations, relative local wrist rotations, and project architecture, see **[TELEOP_ARCHITECTURE.md](./TELEOP_ARCHITECTURE.md)**.

---

## 🛠️ Setup & Running

You will need **4 separate terminal windows** to run the complete pipeline.

### Terminal 1: ROS 2 TCP Endpoint (Bridges Headset to PC)
This node runs on the host and opens a TCP server on port `10000` to receive tracking telemetry from the Quest 2:
```bash
export ROS_DOMAIN_ID=57
source /opt/ros/humble/setup.bash
source ~/projects/LearnROS2/ros2_ws/install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

### Terminal 2: MoveIt Simulation Server (Opens RViz Sim)
This command starts the ROS 1 master, planning scene monitor, and the RViz GUI inside the Docker container:
```bash
docker exec -it sda10-noetic bash -c "source devel/setup.bash && roslaunch sda10_dx100_bringup sda10_moveit_demo.launch"
```

### Terminal 3: MoveIt Client (IK Solver & State Publisher)
This script receives offsets over UDP, queries Inverse Kinematics locally at 30Hz, and updates the joints directly:
```bash
docker exec -it sda10-noetic bash -c "export PYTHONUNBUFFERED=1 && source devel/setup.bash && rosrun sda10_dx100_bringup motoman_moveit_client.py"
```

### Terminal 4: Quest Pose Publisher (VR Processor & Streamer)
This ROS 2 node maps relative controller movement and streams payloads over UDP:
```bash
export ROS_DOMAIN_ID=57
source /opt/ros/humble/setup.bash
source ~/projects/LearnROS2/ros2_ws/install/setup.bash
export PYTHONUNBUFFERED=1
ros2 run q2r2_bringup quest_pose_publisher --ros-args -p scale_factor:=1.8
```

---

## 🎮 How to Teleoperate

1. **Start the Terminals**: Run all 4 terminal commands listed above.
2. **Setup RViz Display**:
   * Click **Add** (bottom left in RViz).
   * Select **Marker**.
   * Verify it is subscribed to `/q2r_target_marker`.
   * Expand the Marker details, go to **Namespaces**, and check `teleop_left_sphere` and `teleop_right_sphere`.
3. **Home Pose (B/Y Buttons)**:
   * When teleop is **INACTIVE** (Red text in RViz), press **B** (Right hand) or **Y** (Left hand) to smoothly return that arm to its starting neutral home pose.
4. **Interactive Anchoring (A/X Buttons)**:
   * Hold your controllers in a comfortable neutral pose and press **A** (Right controller) or **X** (Left controller) once.
   * The status will turn **Green (Teleop: ACTIVE)**, and the robot arm will anchor onto your exact controller position with **zero jumps**.
   * Press **A/X** again to pause tracking at any time.
