"""
Shared defaults for DARO launch files.

Change values here to update all launch files at once.
Individual launch args can still override at runtime, e.g.:
  ros2 launch daro_bringup daro.launch.py baud:=921600
"""

# Serial communication with ESP32
ESP_PORT = "/dev/esp32"
BAUD = "115200"
