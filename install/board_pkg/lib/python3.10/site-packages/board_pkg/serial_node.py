import rclpy
from rclpy.node import Node
import serial
import json
from serial import Serial
from serial.serialutil import SerialException, PortNotOpenError
from sensor_msgs.msg import JointState


class SerialNode(Node):
    def __init__(self, name: str, port: str, baudrate: int, bytesize: int = 8, stopbits: int = 1, parity: str = "N"):
        super().__init__(name)
        self.get_logger().info('Serial node started')
        self.uart_handle: Serial = serial.Serial()
        self.uart_handle.port = port
        self.uart_handle.baudrate = baudrate
        self.uart_handle.bytesize = bytesize
        self.uart_handle.stopbits = stopbits
        self.uart_handle.parity = parity
        self.target_speed: list[int] = [0, 0, 0, 0]
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscriber = self.create_subscription(JointState, 'target_linear_velocity', self.linear_control_callback, 10)
        if not self.open():
            self.get_logger().error(f'Failed to open serial port {self.uart_handle.port}')
        self.timer = self.create_timer(0.025, self.timer_callback)
        

    def open(self) -> bool:
        try:
            self.uart_handle.open()
        except SerialException:
            self.get_logger().error('Failed to open serial port')
            return False
        if not self.uart_handle.is_open:
            self.get_logger().error('Failed to open serial port')
            return False
        else:
            self.get_logger().info(f'Serial port {self.uart_handle.port} opened')
            return True
        
    def send(self, data: str):
        try:
            self.uart_handle.write(data.encode("utf-8"))
        except PortNotOpenError as error:
            self.get_logger().error(f"{error}")

    def timer_callback(self):
        try:
            line = self.uart_handle.readline()
        except PortNotOpenError as error:
            self.get_logger().error(f"{error}")
        else:
            if line != b'':
                try:
                    data = json.loads(line.decode("utf-8").strip())
                    joint_state = JointState()
                    joint_state.name = ["motor_1", "motor_2", "motor_3", "motor_4"]
                    joint_state.position = [float(data["angle"][0]), float(data["angle"][1]), float(data["angle"][2]), float(data["angle"][3])]
                    joint_state.velocity = [float(data["speed"][0]), float(data["speed"][1]), float(data["speed"][2]), float(data["speed"][3])]
                    self.publisher.publish(joint_state)
                except json.JSONDecodeError:
                    self.get_logger().info(f'Received invalid data: {line}')
                else:
                    self.get_logger().info(f'stm32 data: {json.dumps(data)}')


    def linear_control_callback(self, msg: JointState):
        self.target_speed = [int(msg.velocity[0]), int(msg.velocity[1]), int(msg.velocity[2]), int(msg.velocity[3])]
        self.get_logger().info(f"controller data: target_speed={self.target_speed}")
        self.send(f"[{self.target_speed[0]},{self.target_speed[1]},{self.target_speed[2]},{self.target_speed[3]}]")
    
    

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode("serial_node", "/dev/ttyUSB0", 9600)
    rclpy.spin(node)
    rclpy.shutdown()
