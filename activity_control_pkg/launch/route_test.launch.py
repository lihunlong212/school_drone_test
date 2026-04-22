from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_test_node",
            name="route_test_node",
            output="screen",
            parameters=[
                {
                    "map_frame": "map",
                    "laser_link_frame": "laser_link",
                    "output_topic": "/target_position",
                    "position_tolerance_cm": 8.0,
                    "yaw_tolerance_deg": 6.0,
                    "height_tolerance_cm": 5.0,
                    "start_x_cm": 0.0,
                    "start_y_cm": 0.0,
                    "cruise_height_cm": 130.0,
                    "height_band_cm": 20.0,
                    "landing_x_cm": 250.0,
                    "landing_y_cm": 250.0,
                    "mission_yaw_deg": 0.0,
                    "hover_time_sec": 5.0,
                    "pillar_height_threshold_cm": 40.0,
                }
            ],
        )
    ])
