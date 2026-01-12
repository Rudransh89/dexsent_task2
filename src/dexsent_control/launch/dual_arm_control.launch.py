from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 1. Get URDF
    urdf_file = os.path.expanduser('~/dexsent_ws/src/dexsent_description/urdf/dual_box.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 2. Define Parameters DIRECTLY (No external YAML file to lose)
    controller_params = {
        'update_rate': 10,  # Hz
        'joint_state_broadcaster': {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'
        },
        'left_arm_controller': {
            'type': 'forward_command_controller/ForwardCommandController',
            'joints': ['left_joint_1'],
            'interface_name': 'position'
        },
        'right_arm_controller': {
            'type': 'forward_command_controller/ForwardCommandController',
            'joints': ['right_joint_1'],
            'interface_name': 'position'
        }
    }

    return LaunchDescription([
        # 3. Start Controller Manager with embedded parameters
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_desc}, controller_params],
            output='screen'
        ),
        
        # 4. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        # 5. RViz
        Node(
            package='rviz2', executable='rviz2',
            output='screen'
        ),

        # 6. Spawners (Delayed slightly to ensure Manager is ready)
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['left_arm_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['right_arm_controller']),
    ])
