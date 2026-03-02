# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DARO (Roshi's Car) is a multi-platform robotics project with three layers:
- **ESP32 firmware** (C++/Arduino) - motor control, sensors, actuators
- **Raspberry Pi middleware** (Python/ROS2 Jazzy) - serial bridge, input handling, sensor processing
- **macOS dev environment** (Docker) - remote ROS2 monitoring

The ESP32 communicates with the Pi over serial using a newline-delimited JSON (NDJSON) protocol. The Pi runs ROS2 nodes that bridge serial messages to ROS topics.

## Build & Run Commands

### ESP32 (PlatformIO)

```bash
cd esp32
pio run -e main                    # Build main firmware
pio run -e main -t upload          # Build and upload to board
pio device monitor                 # Serial monitor (115200 baud)
pio run -e test_<name> -t upload   # Build and upload a specific test
```

Test environments: `test_led`, `test_tof`, `test_drive_all`, `test_drive_arcade`, `test_drive_protocol`, `test_multi_sensor_i2cbus`, `test_motor_encoder`, `test_motor_drivebase`

### ROS2 (Raspberry Pi)

```bash
cd pi/daro_ros2_ws
colcon build --symlink-install
source install/setup.bash

# Drive only (no SLAM)
ros2 launch daro_bringup daro.launch.py esp_port:=/dev/ttyUSB0

# SLAM — build a map while driving
ros2 launch daro_slam slam.launch.py

# Simulation — Gazebo, no physical hardware needed
ros2 launch daro_sim sim.launch.py
ros2 launch daro_sim sim.launch.py rviz:=true

# Autonomous navigation with a saved map
ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml

# Save a map after driving with SLAM
ros2 run nav2_map_server map_saver_cli -f ~/Code/Roshis-car/pi/daro_ros2_ws/src/daro_nav/maps/my_map
```

Individual bringup launch files: `esp32_bridge.launch.py`, `manual_drive.launch.py`, `tof_test.launch.py`

### Python App (Raspberry Pi)

```bash
cd pi
python app/runners/xbox_to_esp32.py
```

Configuration via environment variables in `pi/app/config.py` (e.g., `SERIAL_PORT`, `SERIAL_BAUD`).

### macOS ROS2 Listener

```bash
cd mac
docker compose up -d
docker exec -it ros_listener bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

## Architecture

### Data Flow

```
Xbox Controller -> Pi (ROS2 joy) -> twist_to_drv_node -> ndjson_bridge -> Serial -> ESP32
ESP32 sensors -> Serial -> ndjson_bridge -> ROS2 topics -> Pi nodes
ESP32 encoders -> enc_json_node -> wheel_odom_node -> EKF -> SLAM Toolbox -> /map
```

### ESP32 Firmware (`esp32/`)

- `src/main.cpp` - Entry point; initializes drivetrain, protocol, I2C bus, sensors, bumper; runs the main polling loop with 500ms drive timeout failsafe
- `lib/core/robot_config.h` - **All GPIO pin assignments and hardware config live here**. Centralized so test environments work without code changes
- `lib/protocol/` - NDJSON serial protocol. Message types: `hello`, `ping/pong`, `drv` (throttle/steering), `feed`, `tof`, `sensor_*`, `err`
- `lib/drive/drivetrain.h` - Arcade drive (throttle + steering) for L9110 motor drivers
- `lib/drive/motor.h` - Newer motor class for DRV8871 drivers with encoder feedback
- `lib/io/i2c_bus.h` - I2C bus abstraction with recovery
- `lib/sensors/tof/tof_manager.h` - VL53L0X time-of-flight sensor array manager
- `lib/io/feeder.h` - Servo-based feeder control
- `lib/io/led_controller.h` - WS2812 LED ring (32 LEDs, FastLED)

Tests are standalone programs in `src/test/` selected via PlatformIO build environments (not a unit test framework).

### ROS2 Packages (`pi/daro_ros2_ws/src/`)

#### Core / Hardware
- `daro_ndjson_bridge` - Bridges ESP32 serial to ROS2 topics. Publishes to `/esp32/rx_json/<type>`, subscribes to `/esp32/tx_json`. Kills any stale process holding the port on launch.
- `daro_actuation` - `twist_to_drv_node`: converts Twist messages to `drv2` drive commands
- `daro_inputs` - `joy_buttons_node`: maps game controller buttons to actions
- `daro_sensors` - Sensor bridge nodes: `enc_json_node` (encoders → JointState), `imu_json_node` (IMU → sensor_msgs/Imu), `tof_to_range_node` (ToF → Range), `wheel_odom_node` (JointState → nav_msgs/Odometry)
- `daro_bringup` - Top-level launch files and YAML config in `config/`. `defaults.py` is the single source for `ESP_PORT` and `BAUD`.
- `daro_description` - Robot URDF (`urdf/daro_min.urdf`) — the single source of truth for physical geometry and sensor positions. See `daro_description/README.md` for full details.

#### SLAM
- `daro_slam` - Map-building stack. Key launch files:
  - `slam.launch.py` — full real-robot SLAM stack (daro_bringup + lidar + EKF + SLAM Toolbox)
  - `slam_core.launch.py` — shared core: `robot_state_publisher`, SLAM Toolbox lifecycle node
  - `localization.launch.py` — EKF node (`robot_localization`) fusing `/wheel/odom` + `/imu/data_raw` → `odom→base_link` TF
  - `lidar.launch.py` — SLLIDAR hardware driver → `/scan`
  - `config/ekf.yaml` — EKF sensor fusion config
  - `config/slam.yaml` — SLAM Toolbox tuning

#### Navigation
- `daro_nav` - Autonomous navigation with a pre-built map. Key files:
  - `launch/nav.launch.py` — full nav stack (hardware + lidar + EKF + AMCL + Nav2 planners)
  - `launch/nav2_stack.launch.py` — Nav2 lifecycle nodes only (AMCL, map server, planners, controller)
  - `config/nav2_params.yaml` — all Nav2 tuning parameters
  - `maps/` — saved maps go here (`.pgm` + `.yaml` pairs)

#### Simulation
- `daro_sim` - Gazebo Harmonic simulation, no physical hardware needed. Key files:
  - `launch/sim.launch.py` — Gazebo world + robot spawn + gz_bridge + SLAM Toolbox
  - `worlds/test_world.sdf` — test environment (boxes, walls, pillar)
  - `config/gz_bridge.yaml` — topic bridge config between Gazebo and ROS2 (`/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, `/joint_states`)

### Python App (`pi/app/`)

Legacy (pre-ROS2) control layer with direct serial communication. Key modules:
- `protocol/ndjson_serial.py` - Serial protocol implementation
- `drive/` - Xbox controller to drive mapping with exponential curves and slew limiting
- `speech/` - Ollama LLM integration and Piper TTS

## Key Conventions

- **Pin changes**: Always update `esp32/lib/core/robot_config.h` — never hardcode GPIO pins elsewhere
- **Protocol messages**: JSON with `"type"` field. Drive: `{"type":"drv2","left":-1.0..1.0,"right":-1.0..1.0}`
- **Serial baud**: 115200. ESP32: `robot_config.cpp` (`SERIAL_BUAD_RATE`). Pi: `daro_bringup/defaults.py` (`BAUD`) — all launch files import from there. Override at runtime: `baud:=460800`
- **ROS2 domain**: `ROS_DOMAIN_ID=27` with static peer discovery to `192.168.7.74` (Pi IP)
- **Drive values**: Differential drive, left/right normalized to -1.0..1.0
- **Motor drivers**: L9110 (old drivetrain, commented out in main.cpp), DRV8871 (current — `motor.h` with encoder feedback)
- **Robot model**: Edit `daro_description/urdf/daro_min.urdf` for any physical changes. If wheel geometry changes, also update `daro_bringup/config/wheel_odom.yaml`. See `daro_description/README.md`.
- **Encoder signs**: `left_sign`/`right_sign` in `daro_bringup/config/daro.yaml` correct for reversed motor/encoder wiring without code changes
