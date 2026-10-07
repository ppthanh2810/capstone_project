                           ┌─────────────────────┐
                           │   Encoder bánh xe   │
                           └──────────┬──────────┘
                                      │
                              /wheel_feedback
                                      │
                                      v
┌─────────────────┐        ┌─────────────────────┐        ┌─────────────────────┐
│     LiDAR C1    │        │   Wheel odometry    │        │    IMU qua ESP32    │
└────────┬────────┘        └──────────┬──────────┘        └──────────┬──────────┘
         │                           │                              │
       /scan                    /wheel_odom                     /imu_odom
         │                           │                              │
         │                           │         ┌────────────────────┘
         v                           v         v
┌─────────────────┐              ┌─────────────────┐    ┌─────────────────────────┐
│    Bộ lọc góc   │              │       EKF       │    │         URDF và         │
└────────┬────────┘              └────────┬────────┘    │  robot_state_publisher  │
         │                                │             └────────────┬────────────┘
         │                                │                          │
  /scan_filtered             TF odom → base_footprint     TF robot → laser_frame
         │                                │                          │
         └─────────────────────┐          │          ┌───────────────┘
                               │          │          │
                               v          v          v
                           ┌─────────────────────────────┐
                           │        slam_toolbox         │
                           └───────┬─────────────┬───────┘
                                   │             │
                                 /map      TF map → odom
                                   │             │
                                   v             v
                           ┌─────────────────────────────┐
                           │     RViz và lưu bản đồ      │
                           └─────────────────────────────┘
                           
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  scan.launch.py
  
  robot_launch = robot_share / "launch" / "display.launch.py" --> "robot_description"
  c1_launch = lidar_share / "launch" / "sllidar_c1_launch.py" --> "sllidar_ros2"
  filter_config = stage2_share / "config" / "angular_filter.yaml" --> "stage2_slam"
  
  
  base_footprint → base_link → lidar_link → laser_frame
  
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
  
  
  mapping.launch.py
  
  --> sử dụng scan.launch.py kết hợp thuật toán slam
  
  include_launch(
    "stage2_slam",
    "scan.launch.py",
    {
        "serial_port": LaunchConfiguration("serial_port"),
        "rviz": LaunchConfiguration("rviz"),
    },
  )
  
  Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    name="slam_toolbox",
    output="screen",
  
  
  parameters=[
    str(
        slam_share
        / "config"
        / "mapper_params_online_async.yaml"
    ),
    {
        "use_sim_time": False,
        "scan_topic": "/scan_filtered",
        "map_frame": "map",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "mode": "mapping",
        "min_laser_range": 0.10,
        "max_laser_range": 16.0,
    },
  ],
  
  
  --> khỏi động xe stage0
  
  include_launch(
    "stage0_ps4_control",
    "move_twist_mux.launch.py",
  )
  
  --> khởi động stage1:
  include_launch(
    "stage1_odom",
    "ekf.launch.py",
  )
  
 -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
 
  localiztion.launch.py
  
  amcl_config = stage2_share / "config" / "amcl_params.yaml"

  default_map = (
      Path.home()
      / "capstone_ws"
      / "capstone_project"
      / "maps"
      / "map.yaml"
  )
  
  --> map server và amcl
  
  include_launch(
    "nav2_bringup",
    "localization_launch.py",
    {
        "map": LaunchConfiguration("map"),
        "params_file": str(amcl_config),
        "use_sim_time": "false",
        "autostart": "true",
        "use_composition": "False",
    },
)
  
 
  
