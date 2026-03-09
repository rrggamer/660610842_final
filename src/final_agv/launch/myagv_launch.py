from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_agv',
            executable='final_sender',
            name='udp_sender',
            output='screen'  
        ),
        Node(
            package='final_agv',
            executable='final_tracker',
            name='gesture_tracker', 
            output='screen'  
        ),
        Node(
            package='final_agv',
            executable='udp_alarm',
            name='udp_alarm_listener',
            output='screen'
        )
    ])