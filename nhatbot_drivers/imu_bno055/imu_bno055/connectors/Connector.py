from rclpy.node import Node
import sys

from imu_bno055.connectors.uart import UART

class Connector:
    """
    Parent class for bno055 connectors
    This class does not contain protocol-specific code for UART
    """
    def __init__(self, node:Node):
        self.node = node
        self.uart = UART(self,
                        115200,
                        '/dev/ttyUSB0',
                        0.1)

    def receive(self, reg_addr, length):
        return self.uart.read(reg_addr, length)
    
    def transmit(self, reg_addr, length, data:bytes):
        return self.uart.write(reg_addr, length, data)