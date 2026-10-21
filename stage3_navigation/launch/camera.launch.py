from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# M2: camera RealSense D435 cho lớp hợp nhất camera + LiDAR (config/camera_fusion.yaml).
# CẢNH BÁO: mở thiết bị USB thật (camera). Được navigation.launch.py include khi camera:=true.
# - publish_tf:=false: URDF (robot_state_publisher) đã có camera_link
#   -> camera_{color,depth}_optical_frame;
#   driver publish TF sẽ trùng cạnh TF.
# - Point cloud KHÔNG texture (pointcloud.stream_filter 0): giữ toàn bộ FOV độ sâu 87°
#   (texture màu chỉ 69°).
#   Topic: /camera/camera/depth/color/points, frame camera_depth_optical_frame.
# - Độ sâu 424x240 @15 fps + decimation 2 -> ~212x120 = ~25k điểm/frame.
def generate_launch_description():
    rs_launch = Path(get_package_share_directory("realsense2_camera")) / "launch" / "rs_launch.py"
    return LaunchDescription([
        DeclareLaunchArgument(
            "depth_profile",
            default_value="424,240,15",
            description="Độ sâu D435: rộng,cao,fps",
        ),
        DeclareLaunchArgument(
            "enable_color",
            default_value="false",
            description="Bật ảnh màu (để xem trong RViz / YOLO ở P3; M2 chỉ dùng độ sâu)",
        ),
        DeclareLaunchArgument(
            "color_profile",
            default_value="424,240,15",
            description="Ảnh màu D435: rộng,cao,fps (nhỏ để giảm tải USB)",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rs_launch)),
            launch_arguments={
                "camera_namespace": "camera",
                "camera_name": "camera",
                "publish_tf": "false",
                "enable_depth": "true",
                "enable_color": LaunchConfiguration("enable_color"),
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                "pointcloud.enable": "true",
                "pointcloud.stream_filter": "0",
                "pointcloud.allow_no_texture_points": "true",
                "decimation_filter.enable": "true",
                "decimation_filter.filter_magnitude": "2",
                "align_depth.enable": "false",
            }.items(),
        ),
    ])
