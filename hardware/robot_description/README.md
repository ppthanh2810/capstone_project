# AMR `robot_description` — ROS 2 Humble

This ROS 2 `ament_cmake` description package is built from `cad/robot_desciption.scad` (original spelling preserved). The OpenSCAD file remains unchanged. The URDF converts its CAD dimensions from **mm** to **m** and angles to **rad**. Frames use **+X forward, +Y left, +Z up**.

## Install and display

Copy the entire `robot_description/` directory into the **same source folder as your other ROS 2 packages**, e.g. `~/capstone_ws/capstone_project/robot_description/` if `capstone_project` is your workspace source folder. From `~/capstone_ws` run:

```bash
colcon build --packages-select robot_description --symlink-install
source install/setup.bash
ros2 launch robot_description display.launch.py
```

For TF only, without starting RViz:

```bash
ros2 launch robot_description display.launch.py rviz:=false
ros2 run tf2_ros tf2_echo base_footprint imu_link
ros2 run tf2_tools view_frames
```

Start **one** `robot_state_publisher` for this model. For **real hardware**, do not have `joint_state_publisher` invent wheel positions:

```bash
ros2 launch robot_description display.launch.py publish_default_joint_states:=false
```

Your motor/odometry node must publish `sensor_msgs/msg/JointState` on `/joint_states`, with the names `left_wheel_joint`, `right_wheel_joint` and actual position values in **radians**. It must separately publish **`odom -> base_footprint`** as TF; localization can publish **`map -> odom`**. Neither transform is hard-coded in the URDF. If you have a controller publishing `odom -> base_link` instead, change that integration so you have only ONE odometry-to-robot path (or choose a different root design).

Set sensor message `header.frame_id` to `imu_link`, `laser_frame`, `camera_color_optical_frame`, or `camera_depth_optical_frame` as applicable. Frames alone do not publish IMU/scan/image data or wheel odometry.

## TF tree

```text
map  --[localization, optional]-->  odom  --[odometry, external]-->  base_footprint
                                                                    |
                                        base_footprint_joint (fixed, +0.1135 m Z)
                                                                    |
                                                                base_link
  +-- upper_deck_link
  +-- breadboard_link
  +-- imu_link
  +-- camera_mount_link
  +-- camera_link
  |     +-- camera_color_optical_frame
  |     +-- camera_depth_optical_frame
  +-- lidar_support_link
  +-- lidar_link
  |     +-- laser_frame
  +-- left_motor_link / left_motor_plate_link
  +-- right_motor_link / right_motor_plate_link
  +-- left_wheel_link (left_wheel_joint, continuous)
  +-- right_wheel_link (right_wheel_joint, continuous)
  +-- front_caster_link  -- front_caster_wheel_link
  +-- rear_caster_link   -- rear_caster_wheel_link
```

All body, sensor and caster joints are **fixed**. Only the two main wheels are continuous; they require `/joint_states` for their moving TF. Caster wheel spin and caster swivel are not represented because no measured caster joint states were supplied.

## Frame locations (from original OpenSCAD, in `base_footprint`, at zero wheel angles)

| Frame | xyz in meters | Orientation relative to base_footprint |
|---|---|---|
| `base_link` | 0, 0, 0.1135 | identity; origin at bottom face of bottom deck |
| `upper_deck_link` | 0, 0, 0.2210 | identity |
| `breadboard_link` | 0.110, 0, 0.2285 | yaw +90 deg |
| `imu_link` | 0.110, 0, 0.2343 | yaw +90 deg |
| `camera_mount_link` | 0.170, 0, 0.2235 | identity |
| `camera_link` | 0.170, 0, 0.2760 | identity |
| `lidar_support_link` | 0, 0, 0.3250 | identity (mesh has CAD world XY vertices) |
| `lidar_link` / `laser_frame` | 0.150, 0, 0.34715 | identity |
| `left_wheel_link` | 0, +0.170, 0.0535 | joint axis (0, -1, 0) |
| `right_wheel_link` | 0, -0.170, 0.0535 | joint axis (0, -1, 0) |
| `front_caster_wheel_link` | +0.068, 0, 0.0375 | fixed caster orientation |
| `rear_caster_wheel_link` | -0.068, 0, 0.0375 | fixed caster orientation |

Wheel diameter = **107 mm**, main wheel track = **340 mm**, chassis diameter = **400 mm**, caster diameter = **75 mm**. All four tires touch the `base_footprint` ground plane at Z=0 in the CAD nominal configuration.

## Assumptions / limits — calibrate before navigation

- Camera optical frames follow standard ROS orientation (`rpy=-pi/2,0,-pi/2`) but are **coincident with the D435 model center**, because lens-specific extrinsics are not specified by CAD. They are placeholders, not calibrated depth/color intrinsics or inter-camera extrinsics.
- The `laser_frame` coincides with the LiDAR CAD model origin. The physical C1 scan plane height and any installation yaw need measurement.
- OpenSCAD STL meshes preserve shape but **do not preserve the per-part OpenSCAD colors**. URDF uses one approximate material for each visual mesh.
- Detailed caster CSG was too complex for this export; caster housings are simplified URDF primitives with wheel position and tire diameter kept from CAD. Do not use this collision model to certify clearance.
- This package is for TF and RViz visualization. The URDF has simple collision meshes/primitives and **no mass/inertia or drivetrain/dynamics/transmission/ros2_control configuration**. Do not treat it as a ready-to-simulate physics model.
- If CAD dimensions change, synchronize URDF transforms and regenerate affected STL meshes; STL is exported in mm and rendered using a **0.001** mesh scale in URDF.
