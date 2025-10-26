from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = PathJoinSubstitution([FindPackageShare('traffic_light_system'), 'config', 'server_params.yaml'])

    return LaunchDescription([
        Node(
            package='traffic_light_system',
            executable='traffic_light_service',
            name='traffic_light_service',
            parameters=[config_file],
        ),
        Node(
            package='traffic_light_system',
            executable='traffic_density_publisher',
            name='traffic_density_publisher'
        ),
        Node(
            package='traffic_light_system',
            executable='traffic_light_visualizer',
            name='traffic_light_visualizer'
        ),
    ])
