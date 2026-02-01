import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        # Load joint names from YAML file
        config_path = os.path.join(
            get_package_share_directory('motor_state_bridge'),
            'config',
            'joint_map.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.joint_names = config['joint_names']

        # Initialize joint positions to 0.0
        self.joint_positions = [0.0] * len(self.joint_names)

        # Subscriptions for first 3 joints (real)
        self.create_subscription(Float32, '/motor1/state', self.cb1, 10)
        self.create_subscription(Float32, '/motor2/state', self.cb2, 10)
        self.create_subscription(Float32, '/motor3/state', self.cb3, 10)

        # Publisher for /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Publish joint states at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def cb1(self, msg):
        self.joint_positions[0] = math.radians(msg.data)

    def cb2(self, msg):
        self.joint_positions[1] = math.radians(msg.data)

    def cb3(self, msg):
        self.joint_positions[2] = math.radians(msg.data)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
