import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

import yaml

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10"},
    {"name": "respawn", "default": "false"},
    {"name": "num_worker_threads", "default": "4"},
    {"name": "camera", "default": "camera"},
    {"name": "camera_link_frame_id", "default": "camera_link"},
    {"name": "color_frame_id", "default": "camera_color_frame"},
    {"name": "depth_frame_id", "default": "camera_depth_frame"},
    {"name": "color_camera_info_url", "default": ""},
    {"name": "depth_camera_info_url", "default": ""},
    {"name": "depth_registration", "default": "false"},
    {"name": "depth_rtsp_element_config", "default": "depth latency=30"},
    {"name": "depth_rtp_depay_element_config", "default": "rtpgstdepay"},
    {"name": "color_rtsp_element_config", "default": "color latency=30"},
    {"name": "color_rtp_depay_element_config", "default": "rtph264depay"},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param["name"], default_value=param["default"]) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param["name"], LaunchConfiguration(param["name"])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, *args, **kwargs):
    # Depth node configuration
    depth_node = launch_ros.actions.Node(
        package="kinova_vision",
        executable="kinova_vision_node",
        name="kinova_vision_depth",
        output="screen",
        parameters=[{
            "camera_type": "depth",
            "camera_name": "depth",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("depth_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://" + LaunchConfiguration("device").perform(context) + "/" + LaunchConfiguration("depth_rtsp_element_config").perform(context) + " ! " + LaunchConfiguration("depth_rtp_depay_element_config").perform(context),
            "frame_id": LaunchConfiguration("depth_frame_id").perform(context),
            "image_topic": "kinova_depth"
        }],
        remappings=[
            ("camera_info", "/kinova_depth/camera_info"),
        ],
    )

    # Color node configuration
    color_node = launch_ros.actions.Node(
        package="kinova_vision",
        executable="kinova_vision_node",
        name="kinova_vision_color",
        output="screen",
        parameters=[{
            "camera_type": "color",
            "camera_name": "color",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("color_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://" + LaunchConfiguration("device").perform(context) + "/" + LaunchConfiguration("color_rtsp_element_config").perform(context) + " ! " + LaunchConfiguration("color_rtp_depay_element_config").perform(context) + " ! avdec_h264 ! videoconvert",
            "frame_id": LaunchConfiguration("color_frame_id").perform(context),
            "image_topic": "kinova_color"
        }],
        remappings=[
            ("camera_info", "/kinova_color/camera_info"),
        ],
    )

    camera_depth_tf_publisher = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["-0.0195", "-0.005", "0", "0", "0", "0", LaunchConfiguration("camera_link_frame_id").perform(context), LaunchConfiguration("depth_frame_id").perform(context)],
    )
    
    camera_color_tf_publisher = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", LaunchConfiguration("camera_link_frame_id").perform(context), LaunchConfiguration("color_frame_id").perform(context)],
    )
    
    return [depth_node, color_node, camera_depth_tf_publisher, camera_color_tf_publisher]


def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function = launch_setup)
    ])
