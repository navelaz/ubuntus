from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puzzlebot_control',
            executable='path_gen',
            name='path_gen_node',
            output='screen'
        ),
        Node(
            package='puzzlebot_control',
            executable='open_ctrl',
            name='open_ctrl_node',
            output='screen'
        )
    ])
