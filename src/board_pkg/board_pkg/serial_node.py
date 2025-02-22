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
        self.target_speed: list[int] = [0, 0, 0, 0]     # Target speed of each motor send to STM32
        self.target_effort: list[int] = [0, 0, 0, 0]    # Target effort of each motor send to STM32
        self.joint_state: JointState = JointState()     # Observed joint states from STM32
        self.joint_state.name = ["motor_1", "motor_2", "motor_3", "motor_4"]
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscriber = self.create_subscription(JointState, 'control_value', self.joint_control_callback, 10)
        if not self.open():
            self.get_logger().error(f'Failed to open serial port {self.uart_handle.port}')
        self.timer = self.create_timer(0.025, self.timer_callback)
        

    def open(self) -> bool:
        """ Open serial port

            Returns:
                bool: True if serial port opened successfully, False otherwise
        """
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
        
    def send(self, data: str) -> None:
        """ Send data to serial port 

            Args:
                data (str): Data to send
        """
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
                    self.joint_state.effort = [float(current) for current in data["current"]]
                    self.joint_state.position = [float(angle) for angle in data["angle"]]
                    self.joint_state.velocity = [float(speed) for speed in data["speed"]]
                    self.publisher.publish(self.joint_state)
                except json.JSONDecodeError:
                    self.get_logger().info(f'Received invalid data: {line}')
                else:
                    self.get_logger().info(f'stm32 data: {json.dumps(data)}')


    def joint_control_callback(self, joint_state_control_value: JointState):
        """ Control joint states

            Args:
                joint_state_control_value (JointState): Joint state control value   
        """
        if joint_state_control_value.header.frame_id == "velocity":
            self.target_speed = [int(velocity) for velocity in joint_state_control_value.velocity]
            self.send(f"[1,{self.target_speed[0]},{self.target_speed[1]},{self.target_speed[2]},{self.target_speed[3]}]")
            self.get_logger().info(f"velocity control: target_speed={self.target_speed}")
        elif joint_state_control_value.header.frame_id == "effort":
            self.target_effort = [int(effort) for effort in joint_state_control_value.effort]
            self.send(f"[0,{self.target_effort[0]},{self.target_effort[1]},{self.target_effort[2]},{self.target_effort[3]}]")
            self.get_logger().info(f"effort control: target_effort={self.target_effort}")
    
   

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode("serial_node", "/dev/ttyUSB0", 9600)
    rclpy.spin(node)
    rclpy.shutdown()
