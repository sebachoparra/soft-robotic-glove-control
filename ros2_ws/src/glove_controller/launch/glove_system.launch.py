"""Launch the three Stage 5 runtime nodes on the frozen ROS graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the bridge, estimator, and control wrapper."""
    transport = LaunchConfiguration('transport')
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='serial'),
        Node(
            package='pico_bridge', executable='pico_bridge_node',
            parameters=[{'transport': transport}],
            remappings=[
                ('~/sensor_frame', '/pico_bridge/sensor_frame'),
                ('~/estimator_input', '/pico_bridge/estimator_input'),
                ('~/actuator_command', '/glove_control/actuator_command'),
            ]),
        Node(
            package='glove_estimator', executable='ukf_node',
            remappings=[
                ('~/estimator_input', '/pico_bridge/estimator_input'),
                ('~/estimator_state', '/ukf/estimator_state'),
            ]),
        Node(
            package='glove_controller', executable='glove_control_node',
            remappings=[
                ('~/sensor_frame', '/pico_bridge/sensor_frame'),
                ('~/estimator_state', '/ukf/estimator_state'),
                ('~/actuator_command', '/glove_control/actuator_command'),
            ]),
    ])
