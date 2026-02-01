from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('manip_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'manipulator_urdf.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    gazebo_ros_path = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gz_sim.launch.py')
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-entity', 'manipulator', '-topic', 'robot_description'],
            output='screen'
        )
    ])

