from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='drone_core',
            executable='altitude_publisher',
            name='altitude_publisher',
            output='screen',
        ),

        Node(
            package='drone_core',
            executable='altitude_subscriber',
            name='altitude_subscriber',
            output='screen',
        ),

    ])
