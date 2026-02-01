from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='motor_state_bridge',
            executable='trajectory_to_esp',
            name='joint_command_relay',
        ),
        Node(
            package='motor_state_bridge',
            executable='bridge_node',
            name='bridge_to_joint_state',
        )
    ])
