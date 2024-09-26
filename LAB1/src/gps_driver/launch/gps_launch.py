from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev//dev/pts/2',
            description='Serial port for GPS device'
        ),
        Node(
            package='gps_driver',
            executable='gps_driver_node',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port')
            }]
        ),
        Node(
            package='gps_driver',
            executable='utm_service_node',
            name='utm_service',
            output='screen',
        ),
    ])
