import sys
import os

import serial
from rclpy.node import Node
from imu_bno055 import registers
from error_handling.exceptions import BusOverRunException, TransmissionException
from connectors.Connector import Connector

class UART():
    """Connector implementation for serial connection to the sensor"""
    CONNECTORTYPE_UART = 'uart'

    def __init__(self, node:Node, baudrate, port, timeout):
        """Initialize the UART
        :param node: A ROS node
        :param baudrate: baudrate to configure UART communication to
        :param timeout: timeout to use
        :return
        """
        #super().__init__(node)
        self.node = node 
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.serialConnection = None 
        #self.serialConnection = serial.Serial()

 
    def connect(self):
        """
        connect to the sensor
        :return
        """
        self.node.get_logger().info('opening serial port: "%s"...' % self.port)
        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as e:
            self.node.get_logger().info('Unable to connect to IMU at port' + self.port)
            self.node.get_logger().info('Check to make sure your devices is connected')
            self.node.get_logger().error('Failed to open serial port: %s' % str(e))
            sys.exit(1)
    
    def read(self, reg_addr, length):
        """
        Read data from sensor via UART
        :param reg_addr: The register address
        :param length: The data length
        :return
        """

        buff_out = bytearray()
        buff_out.append(registers.COM_START_BYTE_WR) # start byte
        buff_out.append(registers.COM_READ) # second byte: Read mode
        buff_out.append(reg_addr) # Reg addr
        buff_out.append(length) # length
        print("buff_out", buff_out)

        try:
            self.serialConnection.write(buff_out)
            buf_in = bytearray(self.serialConnection.read(2 + length))
        except Exception as e:
            raise TransmissionException('Transmission error: %s' % e)
        
        # Check for valid response length (the smallest (error) message has at least 2 bytes)
        if buf_in.__len__()<2:
            raise TransmissionException('Unexpected length of READ-request response: %s'
                                                % buf_in.__len__())
        # Check for READ result (success or failure):
        if buf_in[0] == registers.COM_START_BYTE_ERROR_RESP:
            # Error 0x07 (BUS_OVER_RUN_ERROR) can be "normal" if data fusion is not yet ready
            if buf_in[1] == 7:
                # see #5
                raise BusOverRunException('Data fusion not ready, resend read request')
            else:
                raise TransmissionException('READ-request failed with error code %s'
                                            % hex(buf_in[1]))
        
        # check for correct READ response header:
        if buf_in[0] != registers.COM_START_BYTE_ERROR_RESP:
            raise TransmissionException('Wrong READ-request response header %s ' % hex(buf_in[0]))
        
        if(buf_in.__len__()-2) != buf_in[1]:
            raise TransmissionException('Payload length mismatch detected: '
                                        + ' received=%s, awaited=%s'
                                        %(buf_in.__len__()-2,buf_in[1]))
        
        # check for correct READ-request response length
        if buf_in.__len__() != (2+length):
            raise TransmissionException('Incorrect READ-request response length: %s'
                                        %(2 + length))
        # remove the 0xBB
        buf_in.pop(0)

        # remove the length information
        buf_in.pop(0)

        # return the received payload:
        return buf_in
    
    def write(self,reg_addr, length, data: bytes):
        """
        Transmit data packages to the sensor
        :param reg_addr: The register address
        :param length: The data length
        :return:
        """
        buf_out = bytearray()
        buf_out.append(registers.COM_START_BYTE_RESP)
        buf_out.append(registers.COM_WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out += data

        try:
            self.serialConnection.write(buf_out)
            buf_in = bytearray(self.serialConnection.read(2))
        except Exception: # noqa: B902
            return False
        
        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            return False
        return True
        
        

