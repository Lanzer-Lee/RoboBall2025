import rclpy
from rclpy.node import Node
from serial import Serial
from serial.tools import list_ports
import struct
import math
from sensor_msgs.msg import Imu
import serial


class ImuNode(Node):
    def __init__(self, node_name: str, port: str, baudrate: int = 115200, sample_time: float = 0.001):
        super().__init__(node_name)
        # basic config
        self.port = port
        self.baudrate = baudrate
        self.imu: Serial | None = None
        # data api
        self.key = 0
        self.flag = 0
        self.buffer = {}
        self.angular_velocity: tuple[float] = (0.0, 0.0, 0.0)
        self.acceleration: tuple[float] = (0.0, 0.0, 0.0)
        self.magnetometer: tuple[float] = (0.0, 0.0, 0.0)
        self.angle_degree: tuple[float] = (0.0, 0.0, 0.0)
        # message api
        self.publisher = self.create_publisher(Imu, "imu_data", 10)
        self.timer = self.create_timer(sample_time, self.timer_callback)

    def timer_callback(self):
        self.run()
        msg = Imu()
        msg.angular_velocity.x = float(self.angular_velocity[0])
        msg.angular_velocity.y = float(self.angular_velocity[1])
        msg.angular_velocity.z = float(self.angular_velocity[2])
        msg.linear_acceleration.x = float(self.acceleration[0])
        msg.linear_acceleration.y = float(self.acceleration[1])
        msg.linear_acceleration.z = float(self.acceleration[2])
        msg.orientation.x = float(self.angle_degree[0])
        msg.orientation.y = float(self.angle_degree[1])
        msg.orientation.z = float(self.angle_degree[2])
        self.publisher.publish(msg)
        self.get_logger().info(f"pubish: {msg.orientation.z}, {msg.angular_velocity.z}, {msg.linear_acceleration.z}")

    def search_device(self):
        """
        Search the device.
        """
        ports = [port.device for port in list_ports.comports() if "USB" in port.device]
        self.get_logger().info(f"The computer is connecting with {len(ports)} devices with port: {ports}")
        try:
            self.imu = serial.Serial(self.port, self.baudrate, timeout=0.5)
            if self.imu.is_open:
                self.get_logger().info(f"The device {self.port} is connected!")
            else:
                self.imu.open()
                self.get_logger().warning(f"The device {self.port} is connected!")
        except Exception as error:
            self.get_logger().error(f"The device {self.port} is not connected with error: {error}")
            exit(0)

    def check_sum(self, list_data, check_data) -> bool:
        """
        Check the check sum of the data.

        Args: 
            list_data: The data to check.
            check_data: The check data.
        """
        return sum(list_data) & 0xFF == check_data
    
    def hex_to_short(self, raw_data: bytes):
        return list(struct.unpack("hhhh", bytearray(raw_data)))
    
    def handle_serial_data(self, raw_data):
        """
        Handle the serial data.

        Args:
            raw_data: The raw data.
        """
        angle_flag = False
        self.buffer[self.key] = raw_data
        self.key += 1
        if self.buffer[0] != 0x55:
            self.key = 0
            return
        if self.key < 11:
            return
        else:
            data_buffer = list(self.buffer.values())
            if self.buffer[1] == 0x51:
                if self.check_sum(data_buffer[0:10], data_buffer[10]):
                    self.acceleration = [self.hex_to_short(data_buffer[2:10])[i] / 32768 * 16 * 9.8 for i in range(3)]
                else:
                    self.get_logger().error("Check sum error!")
            elif self.buffer[1] == 0x52:
                if self.check_sum(data_buffer[0:10], data_buffer[10]):
                    self.angular_velocity = [self.hex_to_short(data_buffer[2:10])[i] / 32768 * 2000 * math.pi / 180 for i in range(3)]
                else:
                    self.get_logger().error("Check sum error!")
            elif self.buffer[1] == 0x53:
                if self.check_sum(data_buffer[0:10], data_buffer[10]):
                    self.angle_degree= [self.hex_to_short(data_buffer[2:10])[i] / 32768 * 180 for i in range(3)]
                    angle_flag = True
                else:
                    self.get_logger().error("Check sum error!")
            elif self.buffer[1] == 0x54:
                if self.check_sum(data_buffer[0:10], data_buffer[10]):
                    self.magnetometer = self.hex_to_short(data_buffer[2:10])
                else:
                    self.get_logger().error("Check sum error!")
            else:
                self.buffer = {}
                self.key = 0
                self.get_logger().error("The data is not correct!")
            self.buffer = {}
            self.key = 0
            if angle_flag:
                pass
                # self.get_logger().info(f"Acceleration: {self.acceleration}")
                # self.get_logger().info(f"Angular velocity: {self.angular_velocity}")
                # self.get_logger().info(f"Magnetometer: {self.magnetometer}")
                # self.get_logger().info(f"Angle degree: {self.angle_degree}")

    def run(self):
        try:
            buffer_count = self.imu.in_waiting
        except Exception as error:
            self.get_logger().error(f"The device {self.port} is not connected with error: {error}")
            exit(0)
        else:
            if buffer_count > 0:
                buffer_data = self.imu.read(buffer_count)
                for i in range(buffer_count):
                    self.handle_serial_data(buffer_data[i])


def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode("imu_node", "/dev/ttyUSB1")
    imu_node.search_device()
    rclpy.spin(imu_node)
    rclpy.shutdown()

