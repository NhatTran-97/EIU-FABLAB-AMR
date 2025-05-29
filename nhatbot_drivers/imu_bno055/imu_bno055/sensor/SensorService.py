import json
from math import sqrt
import struct
import sys
from time import sleep

from imu_bno055 import registers
from imu_bno055.connectors.Connector import Connector
from imu_bno055.params.NodeParameters import Parameters
from imu_bno055.connectors.uart import UART

from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from example_interfaces.srv import Trigger



class SensorService:
    """Provide an interface for accessing the sensor's features & data """
    def __init__(self,node:Node, connector:Connector, param:Parameters):
        self.uart = UART(self,
                115200,
                '/dev/ttyUSB0',
                0.1)
        self.node = node
        self.con = connector
        self.param = param

        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)

        # create topic publisher:
        self.pub_imu_raw = self.node.create_publisher(Imu,prefix + "imu_raw", QoSProf)
        self.pub_imu = self.node.create_publisher(Imu, prefix + 'imu',QoSProf)
        self.pub_mag = self.node.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_temp = self.node.create_publisher(Temperature,prefix + 'temp',QoSProf)
        self.pub_calib_status = self.node.create_publisher(String, prefix + 'calib_status', QoSProf)
 
    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info('Configuring device...')
        try:
            data = self.uart.read(registers.BNO055_CHIP_ID_ADDR, 1)
            if data[0] != registers.BNO055_ID:
                raise IOError('Device ID=%s is incorrect' % data)
            # print("device sent ", binascii.hexlify(data))
        except Exception as e:  # noqa: B902
            # This is the first communication - exit if it does not work
            self.node.get_logger().error('Communication error: %s' % e)
            self.node.get_logger().error('Shutting down ROS node...')
            sys.exit(1)