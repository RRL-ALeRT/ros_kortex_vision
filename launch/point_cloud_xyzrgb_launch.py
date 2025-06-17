import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

import yaml


def generate_launch_description():
    # launch plugin through rclcpp_components container
    depth_image_proc = launch_ros.actions.ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node",
                remappings=[
                    ("depth/image_rect", "/kinova_depth"),
                    ("depth/camera_info", "/kinova_depth/camera_info"),
                    ("rgb/camera_info", "/kinova_color/camera_info"),
                    ("depth_registered/image_rect","/kinova_depth_registered"),
                    ("depth_registered/camera_info","/kinova_depth_registered/camera_info"),
                ],
            ),
            # launch_ros.descriptions.ComposableNode(
            #     package="depth_image_proc",
            #     plugin="depth_image_proc::PointCloudXyzrgbNode",
            #     name=("point_cloud_xyzrgb"),
            #     remappings=[
            #         ("rgb/camera_info", "/kinova_color/camera_info"),
            #         ("rgb/image_rect_color", "/kinova_color"),
            #         ("depth_registered/image_rect","/kinova_depth_registered"),
            #         ("points", "/kinova_points"),
            #     ],
            # ),
        ],
        output="screen",
    )
    return LaunchDescription([depth_image_proc])
