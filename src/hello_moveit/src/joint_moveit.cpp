#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "joint_space_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("joint_space_demo");

    // Initialize MoveGroupInterface for the arm
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "newmanip_arm");

    // Configure planning parameters
    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setNumPlanningAttempts(10);
    move_group_interface.setGoalJointTolerance(0.01);      // Tight joint tolerance
    move_group_interface.setGoalPositionTolerance(0.02);   // Slightly looser for Cartesian
    move_group_interface.setGoalOrientationTolerance(0.1); // Radians

    // ========== FIRST: JOINT SPACE MOVEMENT ==========
    RCLCPP_INFO(logger, "=== Attempting joint space goal ===");

    // Define a safe joint position (adjust these values for your robot)
    std::vector<double> joint_goal = {0.0, -0.785, 1.57, 0.0, 0.785, 0.0}; // Example for 6-DOF arm

    // Set joint target
    move_group_interface.setJointValueTarget(joint_goal);

    // Plan and execute
    auto [joint_success, joint_plan] = [&move_group_interface]()
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        bool ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (joint_success)
    {
        RCLCPP_INFO(logger, "Joint space planning succeeded! Executing...");
        move_group_interface.execute(joint_plan);

        // Wait for execution to complete
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
    else
    {
        RCLCPP_ERROR(logger, "Joint space planning failed!");
        // Print current joint state for debugging
        auto current_joints = move_group_interface.getCurrentJointValues();
        RCLCPP_INFO(logger, "Current joint values:");
        for (size_t i = 0; i < current_joints.size(); ++i)
        {
            RCLCPP_INFO(logger, "Joint %zu: %.3f", i, current_joints[i]);
        }
    }

    // ========== SECOND: CARTESIAN POSE MOVEMENT ==========
    RCLCPP_INFO(logger, "\n=== Attempting Cartesian pose goal ===");

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.1;
    target_pose.orientation.w = 1.0; // Neutral orientation

    move_group_interface.setPoseTarget(target_pose);

    auto [pose_success, pose_plan] = [&move_group_interface]()
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        bool ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (pose_success)
    {
        RCLCPP_INFO(logger, "Cartesian planning succeeded! Executing...");
        move_group_interface.execute(pose_plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Cartesian planning failed!");

        // Check if IK exists for the target pose
        bool has_ik = move_group_interface.setJointValueTarget(target_pose);
        RCLCPP_INFO(logger, "IK solution exists: %s", has_ik ? "YES" : "NO");

        // Print current pose for reference
        auto current_pose = move_group_interface.getCurrentPose();
        RCLCPP_INFO(logger, "Current end effector pose:");
        RCLCPP_INFO(logger, "Position: [%.3f, %.3f, %.3f]",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);
    }

    rclcpp::shutdown();
    return 0;
}