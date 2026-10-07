# AMR `robot_description` — ROS 2 Humble

This ROS 2 `ament_cmake` description package is built from `cad/robot_desciption.scad` (original spelling preserved). The OpenSCAD file remains unchanged. The URDF converts its CAD dimensions from **mm** to **m** and angles to **rad**. Frames use **+X forward, +Y left, +Z up**.

## Install and display

Copy the entire `robot_description/` directory into the **same source folder as your other ROS 2 packages**, e.g. `~/capstone_ws/capstone_project/hardware/robot_description/` (its location in this repo). From `~/capstone_ws` run:

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

Start **one** `robot_state_publisher` for this model. The URDF has **no movable joints** (all joints are `fixed`; wheels, motors and casters are visual/collision geometry inside `base_link`), so `robot_state_publisher` publishes the whole tree on `/tf_static` and no `/joint_states` are needed. On the real robot run it without `joint_state_publisher` (this is what `stage2_slam/scan.launch.py` does):

```bash
ros2 launch robot_description display.launch.py publish_default_joint_states:=false
```

Odometry must publish **`odom -> base_footprint`** (in this project: `robot_localization` EKF, `stage1_odom/config/ekf.yaml`); localization publishes **`map -> odom`** (slam_toolbox while mapping, AMCL while localizing — never both). Neither transform is in the URDF. Odometry messages use `header.frame_id: odom`, `child_frame_id: base_footprint`. Never publish `odom -> base_link`: `base_link` already has the parent `base_footprint`.

Set sensor message `header.frame_id` to `imu_link`, `laser_frame`, `camera_color_optical_frame`, or `camera_depth_optical_frame` as applicable. Frames alone do not publish IMU/scan/image data or wheel odometry.

## TF tree

```text
map  --[slam_toolbox | AMCL]-->  odom  --[ekf_filter_node]-->  base_footprint
                                                                    |
                                        base_footprint_joint (fixed, +0.1135 m Z)
                                                                    |
                                                                base_link   (also carries deck, motor, wheel, caster visuals)
  +-- imu_link                     (imu_joint, fixed)
  +-- camera_link                  (camera_joint, fixed)
  |     +-- camera_color_optical_frame
  |     +-- camera_depth_optical_frame
  +-- lidar_link                   (lidar_joint, fixed)
        +-- laser_frame            (laser_joint, fixed)
```

All joints are **fixed**. Wheel rotation is not represented in TF.

## Frame locations (from original OpenSCAD, in `base_footprint`)

| Frame | xyz in meters | Orientation relative to base_footprint |
|---|---|---|
| `base_link` | 0, 0, 0.1135 | identity; origin at bottom face of bottom deck |
| `imu_link` | 0.110, 0, 0.2343 | identity (only the IMU board *visual* is yawed +90 deg) |
| `camera_link` | 0.170, 0, 0.2760 | identity |
| `camera_*_optical_frame` | 0.170, 0, 0.2760 | rpy (-pi/2, 0, -pi/2) — REP-103 optical convention |
| `lidar_link` / `laser_frame` | 0.150, 0, 0.34715 | identity |

Visual-only geometry in `base_link` (no frame): drive wheels centred at (0, ±0.170, 0.0535), caster wheels at (±0.068, 0, 0.0375) in `base_footprint`.

Wheel diameter = **107 mm**, main wheel track = **340 mm**, chassis diameter = **400 mm**, caster diameter = **75 mm**. All four tires touch the `base_footprint` ground plane at Z=0 in the CAD nominal configuration.

## Assumptions / limits — calibrate before navigation

- Camera optical frames follow standard ROS orientation (`rpy=-pi/2,0,-pi/2`) but are **coincident with the D435 model center**, because lens-specific extrinsics are not specified by CAD. They are placeholders, not calibrated depth/color intrinsics or inter-camera extrinsics.
- The `laser_frame` coincides with the LiDAR CAD model origin. The physical C1 scan plane height and any installation yaw need measurement.
- OpenSCAD STL meshes preserve shape but **do not preserve the per-part OpenSCAD colors**. URDF uses one approximate material for each visual mesh.
- Detailed caster CSG was too complex for this export; caster housings are simplified visual primitives with wheel position and tire diameter kept from CAD. Do not use this collision model to certify clearance.
- This package is for TF and RViz visualization. The URDF has simple collision meshes/primitives and **no mass/inertia or drivetrain/dynamics/transmission/ros2_control configuration**. Do not treat it as a ready-to-simulate physics model.
- If CAD dimensions change, synchronize URDF transforms and regenerate affected STL meshes; STL is exported in mm and rendered using a **0.001** mesh scale in URDF.
