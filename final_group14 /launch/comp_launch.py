from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_group14',
            executable='start_comp',
            name='commander',
            output='screen',
            parameters=[
                '/home/kiran/group14_ws/src/final_group14/config/params.yaml', 
            ],
        ),
    ])
