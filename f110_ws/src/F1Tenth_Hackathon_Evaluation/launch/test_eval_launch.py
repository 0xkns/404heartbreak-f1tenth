from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='evaluation',
            executable='wall_follow',
        ),
        Node(
            package='evaluation',
            executable='evaluation',
        )
        ]) 
