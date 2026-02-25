# DARO ROS2 Workspace

## Package Overview

| Package | Responsibility |
|---|---|
| `daro_bringup` | Robot hardware — ESP32 bridge, sensors, joystick, drive |
| `daro_slam` | SLAM stack — LiDAR driver, EKF localization, SLAM Toolbox |
| `daro_nav` | Autonomous navigation — Nav2, AMCL, map server |
| `daro_sim` | Gazebo simulation — world, robot spawn, ROS2 bridge |
| `daro_description` | Robot model — URDF, SDF |
| `daro_sensors` | Sensor nodes — ToF, IMU, encoder → ROS messages |
| `daro_actuation` | Drive output — Twist → ESP32 drive commands |
| `daro_inputs` | Controller input — Xbox buttons → ROS events |
| `daro_ndjson_bridge` | Serial protocol — NDJSON bridge between ESP32 and ROS2 |

---

## Build

```bash
cd pi/daro_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Workflows

### 1. Manual Driving

Drive the robot with an Xbox controller. No SLAM, no sensors beyond what the ESP32 provides.

```bash
ros2 launch daro_bringup manual_drive.launch.py
```

**Optional overrides:**
```bash
ros2 launch daro_bringup manual_drive.launch.py esp_port:=/dev/ttyUSB0 baud:=460800
```

**Xbox controls:**
- Left stick Y → forward / backward
- Right stick X → rotate left / right

---

### 2. Build a Map (SLAM)

Bring up the full robot with LiDAR, EKF, and SLAM Toolbox. Drive the robot around to build a map.

**Step 1 — Launch SLAM:**
```bash
ros2 launch daro_slam slam.launch.py
```

**Step 2 — Drive around** to build the map (use the Xbox controller).

**Step 3 — Save the map** when coverage is good:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
# Saves: ~/maps/my_map.yaml + ~/maps/my_map.pgm
```

**Step 4 — Stop** the SLAM launch (`Ctrl+C`).

**Optional overrides:**
```bash
ros2 launch daro_slam slam.launch.py \
  esp_port:=/dev/ttyUSB0 \
  baud:=460800 \
  slam_params:=/path/to/custom_slam.yaml \
  ekf_params:=/path/to/custom_ekf.yaml
```

**View the map:**
```bash
rviz2 -d <path>/daro_bringup/rviz/slam.rviz
```

---

### 3. Autonomous Navigation (Nav2)

Localize in a previously saved map and send navigation goals. Requires a map file from the SLAM workflow above.

**Step 1 — Launch navigation:**
```bash
ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml
```

**Step 2 — Initialize AMCL pose** in RViz:
- Open RViz: `rviz2 -d <path>/daro_bringup/rviz/slam.rviz`
- Click **"2D Pose Estimate"** and click on the map where the robot is

**Step 3 — Send a navigation goal:**
- Click **"2D Nav Goal"** and click the target location on the map
- Nav2 plans a path and drives the robot autonomously

**Optional overrides:**
```bash
ros2 launch daro_nav nav.launch.py \
  map:=/home/pi/maps/my_map.yaml \
  esp_port:=/dev/ttyUSB0 \
  baud:=460800 \
  nav2_params:=/path/to/custom_nav2_params.yaml \
  ekf_params:=/path/to/custom_ekf.yaml
```

**Tune navigation parameters:**
Edit `daro_nav/config/nav2_params.yaml` — all Nav2 settings are in one file:
- `amcl` — localization particle filter, laser model
- `controller_server` → `FollowPath` — max velocity, acceleration limits, DWB critic weights
- `local_costmap` / `global_costmap` — obstacle inflation radius, resolution
- `planner_server` — global path planning tolerance
- `behavior_server` — spin/backup recovery distances and speeds

---

### 4. Simulation (Gazebo)

Run the full SLAM stack in Gazebo Harmonic — no physical hardware needed.

**Step 1 — Launch simulation:**
```bash
ros2 launch daro_sim sim.launch.py
```

**Step 2 — Open Gazebo GUI** (in a separate terminal):
```bash
gz sim -g
```

**Step 3 — View SLAM map in RViz** (optional):
```bash
ros2 launch daro_sim sim.launch.py rviz:=true
# or separately:
rviz2 -d <path>/daro_bringup/rviz/slam.rviz
```

**Drive the simulated robot:**
- Xbox controller works the same as the real robot
- Or use keyboard teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

---

## Launch File Reference

### `daro_bringup`

| Launch file | Command | What it starts |
|---|---|---|
| `daro.launch.py` | *(included by other packages)* | ESP32 bridge, sensors (ToF, IMU, encoders, wheel odom), joystick, drive |
| `esp32_bridge.launch.py` | *(included by daro.launch.py)* | NDJSON serial bridge to ESP32 |
| `manual_drive.launch.py` | `ros2 launch daro_bringup manual_drive.launch.py` | Xbox joystick → drive only, no sensors |
| `tof_test.launch.py` | `ros2 launch daro_bringup tof_test.launch.py` | ESP32 bridge + ToF sensor only, for sensor testing |

### `daro_slam`

| Launch file | Command | What it starts |
|---|---|---|
| `slam.launch.py` | `ros2 launch daro_slam slam.launch.py` | **Full SLAM stack** — hardware + LiDAR + EKF + SLAM Toolbox |
| `lidar.launch.py` | *(included by slam.launch.py, nav.launch.py)* | SLLIDAR driver → `/scan` |
| `localization.launch.py` | *(included by slam.launch.py, nav.launch.py)* | EKF node — fuses `/wheel/odom` + `/imu/data_raw` → `odom→base_link` TF |
| `slam_core.launch.py` | *(included by slam.launch.py, nav.launch.py, sim.launch.py)* | robot_state_publisher, static TFs, SLAM Toolbox lifecycle node |

### `daro_nav`

| Launch file | Command | What it starts |
|---|---|---|
| `nav.launch.py` | `ros2 launch daro_nav nav.launch.py map:=<path>` | **Full navigation stack** — hardware + LiDAR + EKF + RSP/TF + Nav2 + AMCL |
| `nav2_stack.launch.py` | *(included by nav.launch.py)* | Nav2 nodes only — AMCL, map server, planner, controller, BT navigator, lifecycle manager |

### `daro_sim`

| Launch file | Command | What it starts |
|---|---|---|
| `sim.launch.py` | `ros2 launch daro_sim sim.launch.py` | Gazebo world, DARO robot spawn, ROS2 bridge, SLAM Toolbox |

---

## Configuration Files

| File | Package | Tune when you want to... |
|---|---|---|
| `daro.yaml` | `daro_bringup` | Change sensor topics, frame IDs, serial port |
| `wheel_odom.yaml` | `daro_bringup` | Adjust wheel radius, track width |
| `twist_to_drv.yaml` | `daro_bringup` | Tune drive slew rates, deadband, speed scaling |
| `teleop_joy.yaml` | `daro_bringup` | Remap joystick axes / buttons |
| `ekf.yaml` | `daro_slam` | Tune sensor fusion weights and covariances |
| `lidar.yaml` | `daro_slam` | Change LiDAR serial port or scan mode |
| `slam.yaml` | `daro_slam` | Adjust map resolution, scan range, update rate |
| `nav2_params.yaml` | `daro_nav` | Tune everything Nav2 — speeds, costmaps, AMCL, recovery |
| `gz_bridge.yaml` | `daro_sim` | Add/remove Gazebo ↔ ROS2 topic bridges |

---

## Topic Map

```
Xbox controller
  └── joy_node          → /joy
  └── teleop_twist_joy  → /cmd_vel
  └── joy_buttons_node  → /daro/events/*

ESP32 (via ndjson_bridge)
  └── enc_json_node     → /wheel/joint_states
  └── wheel_odom_node   → /wheel/odom
  └── imu_json_node     → /imu/data_raw
  └── tof_to_range_node → /tof/range

/cmd_vel → twist_to_drv_node → /esp32/tx_json → ESP32 motors

LiDAR → sllidar_node → /scan

/wheel/odom + /imu/data_raw → ekf_filter_node → TF: odom→base_link

/scan + TF → slam_toolbox → /map + TF: map→odom
         or → amcl        → TF: map→odom  (navigation mode)

/map + /scan + TF → Nav2 → /cmd_vel  (autonomous navigation)
```

---

## Common Overrides

```bash
# Different serial port
esp_port:=/dev/ttyUSB0

# Different baud rate
baud:=921600

# Custom parameter files
slam_params:=/path/to/slam.yaml
ekf_params:=/path/to/ekf.yaml
nav2_params:=/path/to/nav2_params.yaml

# ROS log level
log_level:=debug
```
