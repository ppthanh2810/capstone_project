"""Lái robot bằng tay PS4 và xem vị trí trong RViz2 bằng wheel odometry (/wheel_odom),
kèm hướng từ IMU (/imu_odom).

!!! Launch này BẬT MOTOR (motor:=true mặc định): zlac8015d_move ghi Modbus lúc khởi động
(disable -> setMode(3) -> enable) trên /dev/ttyUSB0. Kê bánh khỏi mặt đất khi test lần đầu,
để E-stop trong tầm tay. Chỉ xem dữ liệu, không bật motor: motor:=false.

Chạy (không có EKF):
  - stage0 move_twist_mux (motor:=true): joy -> joy_teleop -> twist_mux -> zlac8015d_move -> /wheel_feedback
  - robot_state_publisher (URDF): base_footprint -> base_link -> sensor frames
  - zlac8015d_odom (publish_tf: true): /wheel_feedback -> /wheel_odom + TF odom -> base_footprint
  - imu_odom (imu:=true): đọc ESP32 + BNO055 qua serial (imu_port) -> /imu/data, /imu/mag, /imu_odom
  - rviz2: Fixed Frame = odom, hiển thị RobotModel, TF, vết /wheel_odom và mũi tên yaw IMU

Vị trí và TF vẫn chỉ lấy từ bánh xe; IMU chỉ để so sánh hướng (chưa fuse, muốn fuse dùng ekf.launch.py).
imu_odom chạy với zero_yaw_on_start: true -> yaw IMU lúc khởi động = 0, trùng hướng ban đầu của wheel odom.
Sau khi gọi /reset_odom, gọi thêm /imu_odom/zero_yaw để IMU cũng về 0.

Không chạy cùng ekf.launch.py hoặc move_twist_mux.launch.py riêng lẻ: trùng node,
trùng cổng serial và trùng TF odom -> base_footprint.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = Path(get_package_share_directory("robot_description"))
    stage0_share = Path(get_package_share_directory("stage0_ps4_control"))
    stage1_share = Path(get_package_share_directory("stage1_odom"))

    rviz_config = stage1_share / "rviz" / "odom.rviz"

    return LaunchDescription([
      
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Có mở RViz2 (odom.rviz) hay không",
        ),
        DeclareLaunchArgument(
            "motor",
            default_value="true",
            description="Có chạy stage0 (PS4 + twist_mux + zlac8015d_move, BẬT MOTOR) hay không",
        ),
        DeclareLaunchArgument(
            "imu",
            default_value="true",
            description="Có chạy imu_odom (đọc ESP32 + BNO055) hay không",
        ),
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/ttyUSB1",
            description="Cổng USB-UART của ESP32 (IMU)",
        ),

        # ============================================================
        # STAGE0: PS4 + twist_mux + motor -> /wheel_feedback (BẬT MOTOR)
        # ============================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(stage0_share / "launch" / "move_twist_mux.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("motor")),
        ),

        # ============================================================
        # URDF -> TF base_footprint -> base_link -> sensor frames
        # ============================================================

        # GroupAction(scoped=True): ở Humble, launch_arguments của IncludeLaunchDescription
        # ghi đè launch configuration toàn cục. Không có scope thì "rviz": "false" ở đây
        # cũng tắt luôn RViz wheel_odom bên dưới (cùng tên argument "rviz").
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(robot_share / "launch" / "display.launch.py")
                    ),
                    launch_arguments={
                        # Tắt RViz của robot_description: chỉ mở 1 RViz (odom.rviz).
                        "rviz": "false",
                        # URDF chỉ có joint fixed -> không cần joint_state_publisher (như scan.launch.py).
                        "publish_default_joint_states": "false",
                    }.items(),
                ),
            ],
        ),

        # ============================================================
        # WHEEL ODOMETRY -> /wheel_odom + TF odom -> base_footprint
        # ============================================================

        Node(
            package="stage1_odom",
            executable="zlac8015d_odom",
            name="zlac8015d_odom",
            output="screen",
            parameters=[{
                "wheel_radius": 0.0535,
                "wheel_distance": 0.34,
                "publish_tf": True,
            }],
        ),

        # ============================================================
        # IMU -> /imu/data, /imu/mag, /imu_odom (chỉ đọc serial)
        # ============================================================

        Node(
            package="stage1_odom",
            executable="imu_odom",
            name="imu_odom",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("imu_port"),
                "frame_id": "imu_link",
                "zero_yaw_on_start": True,
            }],
            condition=IfCondition(LaunchConfiguration("imu")),
        ),

        # ============================================================
        # RVIZ2
        # ============================================================

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_wheel_odom",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
