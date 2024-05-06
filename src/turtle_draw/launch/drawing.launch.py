from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_draw', 
            executable='drawing.py',  
            name='drawing_node'  
        )
    ])