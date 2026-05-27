# unknown_PX4-Autopilot

PX4 drone simulation project using **ROS 2 Jazzy**, **Gazebo Harmonic**, **SLAM Toolbox**, and **Nav2**.

This project demonstrates:

- PX4 drone simulation in Gazebo Harmonic
- ROS 2 bridge from Gazebo to ROS 2
- 2D lidar mapping with SLAM Toolbox
- Saved-map navigation with Nav2
- AMCL localization
- `/cmd_vel` to PX4 control using MAVSDK
- Custom Gazebo world: `outdoor_slam_test.sdf`

> PX4-Autopilot is not fully included in this repository.  
> Clone PX4-Autopilot separately, then copy the custom world file from this repository.

---

## System Requirements

Tested with:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- PX4 SITL
- SLAM Toolbox
- Nav2
- MAVSDK Python
- ros_gz_bridge

---

## Repository Structure

```text
unknown_px4_ws/
├── README.md
├── .gitignore
├── px4_custom_files/
│   └── Tools/
│       └── simulation/
│           └── gz/
│               └── worlds/
│                   └── outdoor_slam_test.sdf
└── src/
    └── unknown_px4_mapping/
        ├── config/
        │   ├── nav2_map_params.yaml
        │   ├── nav2_params.yaml
        │   └── slam_toolbox.yaml
        ├── launch/
        │   ├── px4_bridge_odom_tf.launch.py
        │   ├── px4_cmd_vel_to_px4.launch.py
        │   ├── px4_nav2_map.launch.py
        │   └── px4_slam_toolbox.launch.py
        ├── maps/
        │   ├── outdoor_px4_slam_map.yaml
        │   └── outdoor_px4_slam_map.pgm
        ├── package.xml
        ├── setup.py
        └── unknown_px4_mapping/
            ├── cmd_vel_to_px4.py
            └── dynamic_pose_to_odom.py
```

---

# Clone This Repository

```bash
cd ~
git clone https://github.com/wattanatum/unknown_PX4-Autopilot.git unknown_px4_ws
cd ~/unknown_px4_ws
```

---

# Build ROS 2 Workspace

```bash
cd ~/unknown_px4_ws
colcon build --symlink-install
source install/setup.bash
```

To source automatically every new terminal:

```bash
echo "source ~/unknown_px4_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# Install Project Dependencies

This project only needs the ROS 2, Gazebo, Nav2, SLAM Toolbox, bridge, and MAVSDK packages used by the launch files.

## ROS 2 Jazzy packages

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-nav-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-tf2-msgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-venv \
  python3-pip
```

## MAVSDK Python for `cmd_vel_to_px4.py`

```bash
cd ~/unknown_px4_ws
python3 -m venv mavsdk_ros_env
source ~/unknown_px4_ws/mavsdk_ros_env/bin/activate
pip install --upgrade pip
pip install mavsdk
deactivate
```

---

# Install PX4-Autopilot

PX4-Autopilot must be cloned separately.

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

After installation, reboot if PX4 setup asks you to.

---

# Copy Custom Gazebo World to PX4

This repository includes a custom Gazebo world:

```text
px4_custom_files/Tools/simulation/gz/worlds/outdoor_slam_test.sdf
```

Copy it into PX4-Autopilot:

```bash
cp ~/unknown_px4_ws/px4_custom_files/Tools/simulation/gz/worlds/outdoor_slam_test.sdf \
   ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Check:

```bash
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds | grep outdoor_slam_test
```

---

# Run PX4 Gazebo Simulation

Use this when starting the PX4 drone simulation.

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=outdoor_slam_test make px4_sitl gz_x500_lidar_2d
```

Alternative default command:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d
```

Keep this terminal running.

---

# Run ROS 2 Bridge, Odometry, and TF

Open a new terminal:

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_bridge_odom_tf.launch.py
```

This launch file runs:

- `ros_gz_bridge`
- Gazebo lidar scan bridge
- Gazebo dynamic pose bridge
- `/clock` bridge
- `dynamic_pose_to_odom.py`
- static TF `base_link -> link`

Expected TF tree:

```text
odom -> base_link -> link
```

Check `/odom`:

```bash
ros2 topic echo /odom --once
```

Expected:

```text
header.frame_id: odom
child_frame_id: base_link
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Check lidar TF:

```bash
ros2 run tf2_ros tf2_echo base_link link
```

---

# Check Gazebo Bridge Topics

Check `/clock`:

```bash
ros2 topic echo /clock --once
```

Check dynamic pose:

```bash
ros2 topic echo /world/outdoor_slam_test/dynamic_pose/info --once
```

Check lidar scan:

```bash
ros2 topic echo /world/outdoor_slam_test/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan --once
```

Check topic list:

```bash
ros2 topic list
```

---

# SLAM Toolbox Mapping

Use this method when creating a new map.

Do **not** run Nav2 saved-map navigation at the same time as SLAM Toolbox.

## Terminal 1: PX4 Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=outdoor_slam_test make px4_sitl gz_x500_lidar_2d
```

## Terminal 2: SLAM Toolbox

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_slam_toolbox.launch.py
```

## Terminal 3: PX4 `/cmd_vel` Controller

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_cmd_vel_to_px4.launch.py
```

## Terminal 4: RViz2

```bash
source ~/unknown_px4_ws/install/setup.bash
rviz2
```

Open RViz2 config in this path:

```text
~/unknown_px4_ws/src/rviz2/px4_slam_toolbox.rviz
```

---

# Save SLAM Map

After mapping, save the map:

```bash
cd ~/unknown_px4_ws/src/unknown_px4_mapping/maps
ros2 run nav2_map_server map_saver_cli -f outdoor_px4_slam_map
```

Expected files:

```text
outdoor_px4_slam_map.yaml
outdoor_px4_slam_map.pgm
```

Rebuild after saving maps:

```bash
cd ~/unknown_px4_ws
colcon build --packages-select unknown_px4_mapping --symlink-install
source install/setup.bash
```

---

# Nav2 Saved-Map Navigation

Use this method after a map has already been created.

Do **not** run SLAM Toolbox when using saved-map navigation.

## Terminal 1: PX4 Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=outdoor_slam_test make px4_sitl gz_x500_lidar_2d
```

## Terminal 2: Bridge, Odometry, and TF

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_bridge_odom_tf.launch.py
```

Before starting Nav2, check:

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

## Terminal 3: Nav2 with Saved Map

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_nav2_map.launch.py
```

This launch file uses:

```text
config/nav2_map_params.yaml
maps/outdoor_px4_slam_map.yaml
```

## Terminal 4: PX4 `/cmd_vel` Controller

```bash
source ~/unknown_px4_ws/install/setup.bash
ros2 launch unknown_px4_mapping px4_cmd_vel_to_px4.launch.py
```

## Terminal 5: RViz2

```bash
source ~/unknown_px4_ws/install/setup.bash
rviz2
```

Open RViz2 config in this path:

```text
~/unknown_px4_ws/src/rviz2/px4_nav2_map.rviz
```

---

# Check Nav2 Lifecycle

Check if Nav2 nodes are active:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Expected:

```text
active [3]
```

---

# Check Nav2 TF Tree

Nav2 requires:

```text
map -> odom -> base_link -> link
```

Check `odom -> base_link`:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Check `map -> odom`:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Check `map -> base_link`:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Notes:

- `map -> odom` is published by AMCL.
- `odom -> base_link` is published by `dynamic_pose_to_odom.py`.
- `base_link -> link` is published by `static_transform_publisher`.

---

# Check Important ROS 2 Topics

Check map:

```bash
ros2 topic echo /map --once
```

Check AMCL pose:

```bash
ros2 topic echo /amcl_pose --once
```

Check odom:

```bash
ros2 topic echo /odom --once
```

Check Nav2 velocity command:

```bash
ros2 topic echo /cmd_vel
```

Check global plan:

```bash
ros2 topic echo /plan --once
```

Check lidar:

```bash
ros2 topic echo /world/outdoor_slam_test/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan --once
```

---

# Run `cmd_vel_to_px4.py` Directly

Normally use the launch file:

```bash
ros2 launch unknown_px4_mapping px4_cmd_vel_to_px4.launch.py
```

Direct run method:

```bash
source ~/unknown_px4_ws/mavsdk_ros_env/bin/activate

python3 ~/unknown_px4_ws/src/unknown_px4_mapping/unknown_px4_mapping/cmd_vel_to_px4.py \
  --ros-args \
  -p use_sim_time:=true \
  -p auto_arm:=true \
  -p target_altitude:=1.5 \
  -p altitude_kp:=0.18 \
  -p max_vertical_speed:=0.10 \
  -p max_forward_speed:=0.22 \
  -p max_backward_speed:=0.00 \
  -p min_forward_speed:=0.00 \
  -p max_yaw_speed_deg:=20.0 \
  -p min_yaw_speed_deg:=0.0 \
  -p yaw_deadband_deg:=0.5 \
  -p invert_yaw:=true \
  -p smoothing_alpha:=0.20 \
  -p cmd_vel_topic:=/cmd_vel \
  -p path_topic:=/plan \
  -p odom_topic:=/odom \
  -p reset_on_new_path:=false \
  -p reset_if_stuck_after_new_path:=true \
  -p stuck_check_duration:=3.0 \
  -p stuck_movement_threshold:=0.03 \
  -p stuck_cmd_threshold:=0.005 \
  -p stuck_reset_duration:=0.50 \
  -p stuck_reset_cooldown:=5.0
```

---

# Manual Static TF

If `base_link -> link` is missing:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.12 0 0.02 \
  0 0 0 \
  base_link link
```

---

# Manual Bridge Command

Normally use:

```bash
ros2 launch unknown_px4_mapping px4_bridge_odom_tf.launch.py
```

Manual bridge command:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/outdoor_slam_test/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /world/outdoor_slam_test/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

Manual dynamic pose to odom:

```bash
ros2 run unknown_px4_mapping dynamic_pose_to_odom \
  --ros-args \
  -p use_sim_time:=true \
  -p pose_topic:=/world/outdoor_slam_test/dynamic_pose/info \
  -p pose_index:=0
```

---

# Troubleshooting

## Problem: `/odom` does not publish

Check dynamic pose:

```bash
ros2 topic echo /world/outdoor_slam_test/dynamic_pose/info --once
```

Check node:

```bash
ros2 node list | grep dynamic
```

Check topic:

```bash
ros2 topic list | grep odom
```

Run dynamic pose node manually:

```bash
ros2 run unknown_px4_mapping dynamic_pose_to_odom \
  --ros-args \
  -p use_sim_time:=true \
  -p pose_topic:=/world/outdoor_slam_test/dynamic_pose/info \
  -p pose_index:=0
```

---

## Problem: Nav2 says `odom` frame does not exist

Check:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

If missing, start:

```bash
ros2 launch unknown_px4_mapping px4_bridge_odom_tf.launch.py
```

---

## Problem: Nav2 says `map` frame does not exist

AMCL has not published `map -> odom`.

Check:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Fix:

1. Make sure AMCL is running.
2. Set initial pose in RViz2 using **2D Pose Estimate**.
3. Check `/amcl_pose`.

```bash
ros2 topic echo /amcl_pose --once
```

---

## Problem: Behavior plugin error

Correct plugin names use `::`, not `/`.

Correct:

```yaml
plugin: "nav2_behaviors::Spin"
plugin: "nav2_behaviors::BackUp"
plugin: "nav2_behaviors::DriveOnHeading"
plugin: "nav2_behaviors::Wait"
```

Wrong:

```yaml
plugin: "nav2_behaviors/Spin"
```

---

## Problem: Planner plugin error

Correct:

```yaml
plugin: "nav2_navfn_planner::NavfnPlanner"
```

Wrong:

```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"
```

---

## Problem: AMCL says initial pose required

Open RViz2:

```bash
rviz2
```

Set:

```text
Fixed Frame: map
```

Click:

```text
2D Pose Estimate
```

Set the drone position on the map.

---

## Problem: Gazebo is lagging

Close heavy displays in RViz2.

Use lower update rates in Nav2 config:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
```

---

