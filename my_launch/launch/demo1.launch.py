import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    my_carto_pkg_share = FindPackageShare(package="my_carto_pkg").find("my_carto_pkg")
    uart_to_stm32_pkg_share = FindPackageShare(package="uart_to_stm32").find("uart_to_stm32")
    pid_control_pkg_share = FindPackageShare(package="pid_control_pkg").find("pid_control_pkg")
    activity_control_pkg_share = FindPackageShare(package="activity_control_pkg").find("activity_control_pkg")
    pillar_detector_pkg_share = FindPackageShare(package="pillar_detector_pkg").find("pillar_detector_pkg")
    circle_detector_pkg_share = FindPackageShare(package="circle_detector_pkg").find("circle_detector_pkg")

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, "launch", "fly_carto.launch.py")
        )
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, "launch", "uart_to_stm32.launch.py")
        )
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, "launch", "position_pid_controller.launch.py")
        )
    )
    route_target_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, "launch", "route_target_publisher.launch.py")
        )
    )
    pillar_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pillar_detector_pkg_share, "launch", "pillar_detector.launch.py")
        )
    )
    circle_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(circle_detector_pkg_share, "launch", "circle_detector_pkg.launch.py")
        )
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        route_target_publisher_launch,
        pillar_detector_launch,
        circle_detector_launch,
    ])
