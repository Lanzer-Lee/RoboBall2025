import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Literal


class LinearControlNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.target_linear_velocity: float = 0.0
        self.direction: Literal["forward", "backward", "left", "right"] = "forward"
        self.publisher = self.create_publisher(JointState, 'target_linear_velocity', 10)
        self.timer = self.create_timer(0.025, self.timer_callback)

    def timer_callback(self):
        joint_state = JointState()
        joint_state.name = ["motor_1", "motor_2", "motor_3", "motor_4"]
        if self.direction == "forward":
            joint_state.velocity = [self.target_linear_velocity, self.target_linear_velocity, self.target_linear_velocity, self.target_linear_velocity]
        elif self.direction == "backward":
            joint_state.velocity = [-self.target_linear_velocity, -self.target_linear_velocity, -self.target_linear_velocity, -self.target_linear_velocity]
        elif self.direction == "left":
            joint_state.velocity = [-self.target_linear_velocity, self.target_linear_velocity, -self.target_linear_velocity, self.target_linear_velocity]
        elif self.direction == "right":
            joint_state.velocity = [self.target_linear_velocity, -self.target_linear_velocity, self.target_linear_velocity, -self.target_linear_velocity]
        self.publisher.publish(joint_state)
        self.get_logger().info(f"Publishing: {joint_state.velocity}")
        self.simulate()

    def simulate(self):
        self.target_linear_velocity += 1.0

def main(args=None):
    rclpy.init(args=args)
    node = LinearControlNode("linear_control_node")
    rclpy.spin(node)
    rclpy.shutdown()
    