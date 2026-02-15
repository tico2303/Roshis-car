# AGENTS.md
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

Test environments: `test_led`, `test_tof`, `test_drive_all`, `test_drive_arcade`, `test_drive_protocol`, `test_multi_sensor_i2cbus`, `test_motor_encoder`, `test_motor_drivebase`, `test_drivebase_protocol`

### ROS2 (Raspberry Pi)

```bash
cd pi/daro_ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch daro_bringup daro.launch.py esp_port:=/dev/ttyUSB0
```

Individual launch files: `esp32_bridge.launch.py`, `manual_drive.launch.py`, `tof_test.launch.py`

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
```
Note we will be moving towards a fully autonomus implementation, only using the Xbox controller for testing or debugging.

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

- `daro_ndjson_bridge` - Bridges ESP32 serial to ROS2 topics. Publishes to `/esp32/rx_json/<type>`, subscribes to `/esp32/tx_json`
- `daro_actuation` - `twist_to_drv_node`: converts Twist messages to drive commands
- `daro_inputs` - `joy_buttons_node`: maps game controller buttons to actions
- `daro_sensors` - `tof_to_range_node`: converts ToF data to ROS Range messages
- `daro_bringup` - Launch files and YAML config in `config/`

All ROS2 packages use `ament_python` build type.

### Python App (`pi/app/`)

Legacy (pre-ROS2) control layer with direct serial communication. Key modules:
- `protocol/ndjson_serial.py` - Serial protocol implementation
- `drive/` - Xbox controller to drive mapping with exponential curves and slew limiting
- `speech/` - Ollama LLM integration and Piper TTS

## Key Conventions

- **Pin changes**: Always update `esp32/lib/core/robot_config.h` - never hardcode GPIO pins elsewhere
- **Protocol messages**: JSON with `"type"` field. Drive: `{"type":"drv","thr":-100..100,"str":-100..100}`Drive2: `{"type":"drv2","left":-1..1,"right":-1..1}`. The encoders send a Encoder message: `{"type":"enc","seq":13,"t_ms":2,"left":{"dt":35,"tot":2792,"rad_s":16.65996},"right":{"dt":0,"tot":0,"rad_s":0}}`
- **Serial baud**: 115200 everywhere
- **ROS2 domain**: `ROS_DOMAIN_ID=27` with static peer discovery to `192.168.7.74` (Pi IP)
- **Drive values**: Throttle and steering range from -100 to 100
- **Motor drivers**: L9110 (legacy drivetrain in main.cpp), migrating to DRV8871 (newer motor.h with encoders)
