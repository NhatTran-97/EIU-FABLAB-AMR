#!  /usr/bin/python3
import signal
import sys
import os 
import threading
from rclpy.node import Node
import rclpy
from imu_bno055.params.NodeParameters import Parameters
from imu_bno055.connectors.Connector import Connector
from imu_bno055.sensor.SensorService import SensorService
from imu_bno055.error_handling.exceptions import BusOverRunException, TransmissionException
from imu_bno055.connectors.uart import UART

class Bno055Node(Node):
    """
    ROS2 Node for interfacing Bosh Bno055 IMU sensor
    """
    sensor = None
    param = None
    
    def __init__(self):
        super().__init__('bno055')
    def setup(self):
        self.param = Parameters(self)
        if self.param.connection_type.value == UART.CONNECTORTYPE_UART:
            connector = UART(self,
                                self.param.uart_baudrate.value,
                                self.param.uart_port.value,
                                self.param.uart_timeout.value)
        else:
            raise NotImplementedError('unsupported connection type: '
                                                + str(self.param.connection_type.value))
        #connect to BNO055 device
        connector.connect()

        # Instantiate the sensor Service API
        self.sensor = SensorService(self,connector, self.param)
        self.sensor.configure()
       

def main(args=None):

    """
    Main entry method for this ROS2 node
    """
    #try:
    rclpy.init(args=args)
    bno055_node = Bno055Node()
    bno055_node.setup()
    rclpy.spin(bno055_node)
    bno055_node.destroy_node()
    rclpy.shutdown()


    #     lock = threading.Lock()
    #     def read_data():
    #         """Periodic data_query_timer executions to retrieve sensor IMU data"""
    #         if lock.locked():
    #             # critical are still locked
    #             # that means that the previous data query is still processed
    #             bno055_node.get_logger().warn('Message communication in progress - skipping query cycle')
    #             return
    #         lock.acquire()
    #         try:
    #             # perform synchronized block:
    #             bno055_node.sensor.get_sensor_data()
    #         except BusOverRunException:
    #             # data not available yet, wait for next cycle | see #5
    #             return
    #         except ZeroDivisionError:
    #             # division by zero in, return
    #             return
    #         except Exception as e: #noqa: B902
    #             bno055_node.get_logger().warn('Receiving sensor data failed with %s:"%s"' % (type(e).__name__,e))
    #         finally:
    #             lock.release()
    #     def log_calibration_status():
    #         """Periodic logging of calibration data (quality indicators) """
    #         if lock.locked():
    #             # critical area still locked 
    #             # that means that the previous data query is still being processed
    #             bno055_node.get_logger().warn('Message communication in progress - skipping query cycle')
    #             return
    #         # Acquire lock before entering critical area to prevent overlapping data queries
    #         lock.acquire()
    #         try:
    #             # perform synchronized block:
    #             bno055_node.sensor.get_calib_status()
    #         except Exception as e:  # noqa: B902
    #             bno055_node.get_logger().warn('Receiving calibration status failed with %s:"%s"'
    #                                    % (type(e).__name__, e)) 
    #             # traceback.print_exc()
    #         finally:
    #             lock.release()
    #     f = 1.0 / float(bno055_node.param.data_query_frequency.value)
    #     data_query_timer = bno055_node.create_timer(f, read_data)

    #     # start regular calibration status logging
    #     f = 1.0 / float(bno055_node.param.calib_status_frequency.value)
    #     status_timer = bno055_node.create_timer(f, log_calibration_status)
    #     rclpy.spin(bno055_node)

    # except KeyboardInterrupt:
    #     bno055_node.get_logger().info('Ctrl+C received - exiting...')
    #     sys.exit(0)
    # finally:
    #     bno055_node.get_logger().info('ROS node shutdown')
    #     try:
    #         bno055_node.destroy_timer(data_query_timer)
    #         bno055_node.destroy_timer(status_timer)
    #     except UnboundLocalError:
    #         bno055_node.get_logger().info('No timers to shutdown')
        
    #     bno055_node.destroy_node()
    #     rclpy.shutdown()
 

    

    




if __name__ == '__main__':
    main()


