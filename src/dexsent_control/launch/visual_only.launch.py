from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Load the Box URDF
    urdf_file = os.path.expanduser('~/dexsent_ws/src/dexsent_description/urdf/dual_box.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. Robot State Publisher (Calculates the robot shape)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        # 2. RViz (Visualization)
        Node(
            package='rviz2', executable='rviz2',
            output='screen'
        )
    ])
