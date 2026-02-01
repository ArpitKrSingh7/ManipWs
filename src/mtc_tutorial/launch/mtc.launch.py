import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # planning_context with proper configuration
    moveit_config = (
        MoveItConfigsBuilder("newmanip_description", package_name="newmanip_moveit_config")
        .robot_description(file_path="config/newmanip_description.urdf.xacro")
        .robot_description_semantic(file_path="config/newmanip_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Enhanced capability configuration
    moveit_config.move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        "publish_robot_description_semantic": True
    }

    # Start move_group with proper parameters
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.move_group_capabilities,
        ],
    )

    # RViz with complete configuration
    rviz_config_file = os.path.join(
        get_package_share_directory("mtc_tutorial"),
        "rviz",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",  # Changed to screen for debugging
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF with proper coordinate frame setup
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "world","--child-frame-id", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control with proper parameter loading
    ros2_controllers_path = os.path.join(
        get_package_share_directory("newmanip_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Load controllers with delay
    load_controllers = []
    for controller in [
        "newmanip_arm_controller",
        "newmanip_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )