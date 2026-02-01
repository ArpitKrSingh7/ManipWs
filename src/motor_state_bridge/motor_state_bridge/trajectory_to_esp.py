import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class TrajectoryRelay(Node):
    def __init__(self):
        super().__init__('trajectory_to_esp')

        # Load joint_to_topic from YAML
        config_path = os.path.join(
            get_package_share_directory('motor_state_bridge'),
            'config',
            'joint_map.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.joint_to_topic = config.get('joint_to_motor_topic', {})

        # Set up publishers
        self.motor_publishers = {
            joint: self.create_publisher(Float32, topic, 10)
            for joint, topic in self.joint_to_topic.items()
        }

        # Subscribe to trajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

    def trajectory_callback(self, msg):
        if not msg.points:
            self.get_logger().warn("Received trajectory with no points.")
            return

        point = msg.points[0]

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.motor_publishers:
                target_rad = point.positions[i]
                target_deg = math.degrees(target_rad)

                msg_out = Float32()
                msg_out.data = target_deg
                self.motor_publishers[joint_name].publish(msg_out)

                self.get_logger().info(f"Sent {target_deg:.2f}° → {joint_name}")
            else:
                self.get_logger().warn(f"No topic mapped for joint: {joint_name}")


def main():
    rclpy.init()
    node = TrajectoryRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
