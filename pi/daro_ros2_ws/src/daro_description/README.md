# daro_description

Robot model package for DARO. Contains the URDF that defines Daro's physical
geometry and the coordinate frames used by ROS2.

---

## The URDF: `urdf/daro_min.urdf`

This is the **single source of truth** for Daro's physical shape and sensor
positions. Changing a measurement here is all that is needed — no other files
need to be updated.

### Coordinate frame convention

```
X = forward (toward front of robot)
Y = left
Z = up
base_link origin = geometric centre of the chassis footprint, at floor level
```

### What's in the URDF

| Link | Type | Description |
|---|---|---|
| `base_link` | chassis | Box representing the main frame |
| `left_wheel` | continuous joint | Driven wheel, encoder feedback |
| `right_wheel` | continuous joint | Driven wheel, encoder feedback |
| `caster_wheel` | fixed joint | Passive front caster, no encoder |
| `laser` | fixed joint | LiDAR scan origin frame |

### Current measurements

| Part | Measurement | Value |
|---|---|---|
| Chassis length (X) | 8.75" | 0.2223 m |
| Chassis width (Y) | 7.75" | 0.1969 m |
| Wheel diameter | 9 cm | radius = 0.045 m |
| Wheel separation | 9.5" centre-to-centre | 0.2413 m |
| Wheel axle X (from chassis centre) | 7.25" from front on 8.75" car → 2.875" behind centre | -0.0730 m |
| Caster diameter | 1" | radius = 0.0127 m |
| Caster X | front centre of chassis | +0.1112 m |
| LiDAR X | 7.25" from front (same as wheel axle) | -0.0730 m |
| LiDAR Z | 1.6" above base | 0.0406 m |

---

## How the URDF connects to the rest of the stack

`robot_state_publisher` reads the URDF and publishes two kinds of TF transforms:

- **Fixed joints** (`laser_joint`, `caster_joint`) → published once on `/tf_static`
- **Continuous joints** (`left_wheel_joint`, `right_wheel_joint`) → published
  continuously on `/tf` as `/wheel/joint_states` messages arrive from the ESP32

This means the full TF tree is maintained automatically:

```
map
 └── odom          ← SLAM Toolbox publishes this
      └── base_link ← EKF (robot_localization) publishes this
           ├── laser        ← robot_state_publisher (from URDF, fixed)
           ├── caster_wheel ← robot_state_publisher (from URDF, fixed)
           ├── left_wheel   ← robot_state_publisher (from URDF + joint states)
           └── right_wheel  ← robot_state_publisher (from URDF + joint states)
```

No other launch files or config files hardcode frame positions — they all
rely on the URDF via `robot_state_publisher`.

---

## Updating measurements

1. Edit `urdf/daro_min.urdf` — update the `<origin xyz="..."/>` of the
   relevant joint, and update the matching comment block at the top of the file.

2. If wheel geometry changed, also update `daro_bringup/config/wheel_odom.yaml`:
   - `wheel_radius_m`
   - `wheel_separation_m`

3. Rebuild and re-source:
   ```bash
   cd ~/Code/Roshis-car/pi/daro_ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

That's it. No launch files need to change.
