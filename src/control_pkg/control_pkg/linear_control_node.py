import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Literal


class LinearControlNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.control_value: float = self.declare_parameter("control_value", 0.0).value
        self.mode: Literal["velocity", "effort"] = self.declare_parameter("control_mode", "velocity").value
        self.joint_state = JointState()
        self.joint_state.name = ["motor_1", "motor_2", "motor_3", "motor_4"]
        self.direction: Literal["forward", "backward", "left", "right"] = "forward"
        self.publisher = self.create_publisher(JointState, 'control_value', 10)
        self.timer = self.create_timer(0.025, self.timer_callback)

    def timer_callback(self):
        self.control_value = self.get_parameter("control_value").value
        self.mode = self.get_parameter("control_mode").value
        if self.mode == "velocity":
            self.joint_state.header.frame_id = "velocity"
            if self.direction == "forward":
                self.joint_state.velocity = [self.control_value, self.control_value, self.control_value, self.control_value]
            elif self.direction == "backward":
                self.joint_state.velocity = [-self.control_value, -self.control_value, -self.control_value, -self.control_value]
            elif self.direction == "left":
                self.joint_state.velocity = [-self.control_value, self.control_value, -self.control_value, self.control_value]
            elif self.direction == "right":
                self.joint_state.velocity = [self.control_value, -self.control_value, self.control_value, -self.control_value]
            self.get_logger().info(f"Publish velocity: {self.joint_state.velocity}")
        elif self.mode == "effort":
            self.joint_state.header.frame_id = "effort"
            if self.direction == "forward":
                self.joint_state.effort = [self.control_value, self.control_value, self.control_value, self.control_value]
            elif self.direction == "backward":
                self.joint_state.effort = [-self.control_value, -self.control_value, -self.control_value, -self.control_value]
            elif self.direction == "left":
                self.joint_state.effort = [-self.control_value, self.control_value, -self.control_value, self.control_value]
            elif self.direction == "right":
                self.joint_state.effort = [self.control_value, -self.control_value, self.control_value, -self.control_value]
            self.get_logger().info(f"Publish effort: {self.joint_state.effort}")
        self.publisher.publish(self.joint_state)

    def simulate(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LinearControlNode("linear_control_node")
    rclpy.spin(node)
    rclpy.shutdown()
    