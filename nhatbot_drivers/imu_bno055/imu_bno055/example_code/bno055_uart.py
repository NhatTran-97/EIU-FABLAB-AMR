from imu_bno055 import registers
import serial
import sys 
import os
import time
#from imu_bno055.error_handling.exceptions import BusOverRunException, TransmissionException
from error_handling.exceptions import BusOverRunException, TransmissionException

class UART():
    """Connector implementation for serial connection to the sensor"""
    CONNECTORTYPE_UART = 'uart'

    def __init__(self, baudrate, port, timeout):
        """Initialize the UART
        :param node: A ROS node
        :param baudrate: baudrate to configure UART communication to
        :param timeout: timeout to use
        :return
        """
        super().__init__()
  
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.serialConnection = None 
    def connect(self):
        """
        connect to the sensor
        :return
        """
        #self.node.get_logger().info('opening serial port: "%s"...' % self.port)
        print("connecting to uart")
        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print("connected")
        except serial.SerialException:
            self.node.get_logger().info('Unable to connect to IMU at port' + self.port)
            print("Unable to connect to IMU at port' " + self.port)
            print("Check to make sure your devices is connected")
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
    def get_calib_data(self):
        """Read all calibration data"""
        accel_offset_read = self.read(registers.ACCEL_OFFSET_X_LSB_ADDR,6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
        0] # Combine MSB and LSB registers into one decimal

        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
        2] #  # Combine MSB and LSB registers into one decimal

        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
        4]  # Combine MSB and LSB registers into one decimal

        accel_radius_read = self.read(registers.ACCEL_RADIUS_LSB_ADDR, 2)
        accel_radius_read_value = (accel_radius_read[1] << 8) | accel_radius_read[0]

        mag_offset_read = self.read(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_radius_read = self.read(registers.MAG_RADIUS_LSB_ADDR, 2)
        mag_radius_read_value = (mag_radius_read[1] << 8) | mag_radius_read[0]

        gyro_offset_read = self.read(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        calib_data = {'accel_offset': {'x': accel_offset_read_x, 'y': accel_offset_read_y, 'z': accel_offset_read_z}, 'accel_radius': accel_radius_read_value,
                      'mag_offset': {'x': mag_offset_read_x, 'y': mag_offset_read_y, 'z': mag_offset_read_z}, 'mag_radius': mag_radius_read_value,
                      'gyro_offset': {'x': gyro_offset_read_x, 'y': gyro_offset_read_y, 'z': gyro_offset_read_z}}

        return calib_data
    
    def print_calib_data(self):
        """Read all calibration data and print to screen"""
        calib_data = self.get_calib_data()

        print(
            '\tAccel offsets (x,y,z): %d %d %d' %(
                calib_data['accel_offset']['x'],
                calib_data['accel_offset']['y'],
                calib_data['accel_offset']['z']))
        
        print(
            '\tAccel radius: %d' %(
                calib_data['accel_radius'],
            ))
        
        print(
            '\tGyro offsets (x y z): %d %d %d' % (
                calib_data['gyro_offset']['x'],
                calib_data['gyro_offset']['y'],
                calib_data['gyro_offset']['z']))
    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.read(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        print("calib_status_dict: ",calib_status_dict)

        # calib_status_str = String()
        # calib_status_str.data = json.dumps(calib_status_dict)
        # # Publish via ROS topic:
        # self.pub_calib_status.publish(calib_status_str)
    
    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset, mag_radius, acc_radius):
        """
        Write calibration data (define as 16 bit signed hex).

        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        :param mag_radius:
        :param acc_radius:
        """
        # Must switch to config mode to write out
        if not (self.write(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            print("Unable to set IMU into config mode")
        time.sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.write(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset.value[0] & 0xFF]))
            self.write(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset.value[0] >> 8) & 0xFF]))
            self.write(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset.value[1] & 0xFF]))
            self.write(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset.value[1] >> 8) & 0xFF]))
            self.write(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset.value[2] & 0xFF]))
            self.write(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset.value[2] >> 8) & 0xFF]))

            self.write(registers.ACCEL_RADIUS_LSB_ADDR, 1, bytes([acc_radius.value & 0xFF]))
            self.write(registers.ACCEL_RADIUS_MSB_ADDR, 1, bytes([(acc_radius.value >> 8) & 0xFF]))

            self.write(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset.value[0] & 0xFF]))
            self.write(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset.value[0] >> 8) & 0xFF]))
            self.write(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset.value[1] & 0xFF]))
            self.write(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset.value[1] >> 8) & 0xFF]))
            self.write(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset.value[2] & 0xFF]))
            self.write(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset.value[2] >> 8) & 0xFF]))

            self.write(registers.MAG_RADIUS_LSB_ADDR, 1, bytes([mag_radius.value & 0xFF]))
            self.write(registers.MAG_RADIUS_MSB_ADDR, 1, bytes([(mag_radius.value >> 8) & 0xFF]))

            self.write(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset.value[0] & 0xFF]))
            self.write(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset.value[0] >> 8) & 0xFF]))
            self.write(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset.value[1] & 0xFF]))
            self.write(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset.value[1] >> 8) & 0xFF]))
            self.write(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset.value[2] & 0xFF]))
            self.write(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset.value[2] >> 8) & 0xFF]))

            return True
        except Exception:  # noqa: B902
            return False
        
    
    def configure(self):
        """Configure the IMU sensor hardware"""
        #self.node.get_logger().info('Configuring device...')
        print("configurate device")
        try:
            
            data = self.read(registers.BNO055_CHIP_ID_ADDR,1)
            if data[0] != registers.BNO055_ID:
                raise IOError('Device ID=%s is incorrect' % data)
        except Exception as e: #noqa: B902
            # This is the first communication - exit if it does not work
            # self.node.get_logger().error('Communication error: %s' %e)
            # self.node.get_logger().error('Shutting down ROS node...')
            print("Communication error: %s" %e)
            print("Shutting down ROS node...")
            sys.exit(1)

        # IMU connected => apply IMU Configuration
        if not (self.write(registers.BNO055_OPR_MODE_ADDR,1, bytes([registers.OPERATION_MODE_CONFIG]))):
            print("Unable to set IMU into config mode")
        if not (self.write(registers.BNO055_PWR_MODE_ADDR,1,bytes([registers.POWER_MODE_NORMAL]))):
            print("Unable to set IMU normal power mode.")
        if not (self.write(registers.BNO055_PAGE_ID_ADDR,1,bytes([0x00]))):
            print("Unable to set IMU register page 0.")
        if not (self.write(registers.BNO055_SYS_TRIGGER_ADDR,1,bytes([0x00]))):
            print("Unable to start IMU")
        if not (self.write(registers.BNO055_UNIT_SEL_ADDR, 1, bytes([0x83]))):
            print("Unable to set IMU units")


        # The sensor placement configuration (Axis remapping) defines the position and orientation of the sensor mount
        # See also Bosch BNO055 dataset section Remap

        mount_positions = {
            'P0': bytes(b'\x21\x04'),
            'P1': bytes(b'\x24\x00'),
            'P2': bytes(b'\x24\x06'),
            'P3': bytes(b'\x21\x02'),
            'P4': bytes(b'\x24\x03'),
            'P5': bytes(b'\x21\x02'),
            'P6': bytes(b'\x21\x07'),
            'P7': bytes(b'\x24\x05')
        }
        if not (self.write(registers.BNO055_AXIS_MAP_CONFIG_ADDR,2,
                                    mount_positions['P1'])):
            print('Unable to set sensor placement configuration')
        
        # Show the current sensor offsets
        print("Current sensor offsets")
        self.print_calib_data()

        set_offsets = False
        if set_offsets :
            configured_offsets = \
                self.set_calib_offsets(
                    registers.DEFAULT_OFFSET_ACC,
                    registers.DEFAULT_OFFSET_MAG,
                    registers.DEFAULT_OFFSET_GYR,
                    registers.DEFAULT_RADIUS_ACC,
                    registers.DEFAULT_RADIUS_MAG)
            if configured_offsets:
                print("Successfully configured sensor offsets to: ")
                self.print_calib_data()
            else:
                print("setting offsets failed")
        
        # set Device mode
        device_mode = 0x0C

        print("setting device_mode to ", device_mode)

        if not (self.write(registers.BNO055_OPR_MODE_ADDR, 1, bytes([device_mode]))):
            print('Unable to set IMU operation mode into operation mode')
        
   
        print("Bosch BNo055 IMU configuration complete")
    def getSensor(self):
        buf = self.read(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 4)
        print(buf)

if __name__ == '__main__':
    uart = UART(115200,'/dev/ttyUSB0', 0.1)
    uart.connect()
    uart.getSensor()

    #uart.configure()
    
        