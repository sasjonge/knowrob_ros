from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='knowrob_ros',  # Your package name
            executable='knowrob_ros_interface',  # Your node's executable name
            name='knowrob_ros_interface',
            output='screen',  # Output to screen (console logs)
            parameters=[
                {'parameter_name': 'parameter_value'}  # Define parameters if needed
            ]
        ),
    ])
