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
                    # TF 坐标系与输出话题
                    "map_frame": "map",                      # 全局地图坐标系
                    "laser_link_frame": "laser_link",        # 机体定位使用的激光雷达坐标系
                    "output_topic": "/target_position",      # 目标点发布话题

                    # 到点判定阈值
                    "position_tolerance_cm": 8.0,            # 平面位置到点容差，单位 cm
                    "yaw_tolerance_deg": 6.0,                # 偏航角到点容差，单位 deg
                    "height_tolerance_cm": 5.0,              # 高度到点容差，单位 cm

                    # 起飞与巡航参数
                    "start_x_cm": 0.0,                       # 起飞后第一个目标点 X，单位 cm
                    "start_y_cm": 0.0,                       # 起飞后第一个目标点 Y，单位 cm
                    "cruise_height_cm": 130.0,               # 巡航/测柱默认高度，单位 cm
                    "height_band_cm": 20.0,                  # 预留高度带，单位 cm

                    # 降落相关参数
                    "landing_x_cm": 250.0,                   # 降落点 X，单位 cm
                    "landing_y_cm": 250.0,                   # 降落点 Y，单位 cm
                    "mission_yaw_deg": 0.0,                  # 整个任务期望偏航角，单位 deg
                    "landing_align_time_sec": 3.0,           # 进入降落前视觉对准保持时间，单位 s

                    # 柱子测量参数
                    "hover_time_sec": 5.0,                   # 每根柱子悬停测量总时长，单位 s
                    "pillar_height_threshold_cm": 40.0,      # 柱高判定阈值，单位 cm

                    # 圆环视觉对准参数
                    "visual_alignment_tolerance_px": 20.0,   # 视觉对准像素容差，|x|/|y| 小于该值视为对准
                    "visual_alignment_timeout_sec": 0.5,     # 视觉数据最大允许超时，超过后不计入 circle_rank
                }
            ],
        )
    ])
