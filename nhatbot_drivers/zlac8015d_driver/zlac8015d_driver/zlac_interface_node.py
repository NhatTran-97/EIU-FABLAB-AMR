#!/usr/bin/env python3

import numpy as np
from enum import Enum
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from zlac8015d_driver import ZLAC8015D_API  # Import driver ZLAC8015D
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import os 
from nhatbot_msgs.msg import ZlacStatus
from node_parameters import NodeParameters


# Main motor interface node
class Zlac_Interfaces(Node):
    def __init__(self, ):
        super().__init__('zlac_driver_node')

        self.param = NodeParameters(self)

        self.motor_states = ""
        self.ReentGroup = ReentrantCallbackGroup()
        self.bldcMotor = None # Motor interface instance 
        try:
            self.init_system()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to get modbus_port: {str(e)}")
            self.bldcMotor = None
        
        qos = QoSProfile(depth=10)
        qos_cmd = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,depth=1,reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        qos_wheel_jointState = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10,)

        # If motor is connected, setup publishers/subscribers/services/timers
        if self.bldcMotor and self.bldcMotor.is_connected():

            self.wheel_JointState_pub_ = self.create_publisher(JointState, self.param.joint_state_topic, qos_wheel_jointState)
            self.zlac_status_pub_ = self.create_publisher(ZlacStatus, self.param.zlac_status_topic, qos)
            self.wheelVelocities_sub_ = self.create_subscription(Float64MultiArray, self.param.wheel_rotation_topic,  
                                                                 self.sub_Vel_Callback, qos_cmd , callback_group=self.ReentGroup)
            self.timer_JointState_ = self.create_timer(self.param.joint_state_frequency, self.pub_JointState_Callback, callback_group=self.ReentGroup)
            self.timer_zlacStatus_ = self.create_timer(self.param.zlac_status_frequency, self.zlacStatus_Callback, callback_group=self.ReentGroup)
            self.motor_srv_ = self.create_service(Trigger, self.param.reset_encoder_service, self.ResetPos_Callback)
            
    def init_system(self,):
        """Initialize the ZLAC8015D motor interface and configure default parameters"""
        try:
            self.bldcMotor = ZLAC8015D_API(self, self.param)
            if not self.bldcMotor.is_connected():
                raise Exception("Connection to motor failed.")

            # self.bldcMotor.disable_motor()
            self.bldcMotor.set_accel_time(self.param.set_accel_time, self.param.set_accel_time)
            self.bldcMotor.set_decel_time(self.param.set_decel_time, self.param.set_decel_time,)
            self.bldcMotor.set_mode(self.param.set_operation_mode)
            self.bldcMotor.enable_motor()
            self.bldcMotor.set_rpm(self.param.set_stop_rpm, self.param.set_stop_rpm)
            self.get_logger().info("✅ Motor initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"❌ Error during motor initialization: {str(e)}")
            self.bldcMotor = None  

    def zlacStatus_Callback(self):
        """
        Periodically publishes the current ZLAC8015D motor status as a ZlacStatus message.
        Includes battery voltage and (optionally) other motor diagnostics.
        """

        zlac_status = ZlacStatus()
        # motor_temperature = self.bldcMotor.get_motor_temperature()
        zlac_status.battery_voltage = float(self.bldcMotor.get_battery_voltage())
        zlac_status.brake_state = str(self.bldcMotor.get_brake_state())
        zlac_status.control_mode = int(self.bldcMotor.get_mode())
        zlac_status.driver_temp = float(self.bldcMotor.get_driver_temperature())
        zlac_status.vehicle_state = str(self.motor_states)

        self.zlac_status_pub_.publish(zlac_status)

    def pub_JointState_Callback(self):
        """Publish JointState message containing wheel positions and velocities"""

        if not self.bldcMotor or not self.bldcMotor.is_connected():
            return  
        if not rclpy.ok():
            return 
        try:
            msg_wheel_JointState_ = JointState()                   
            msg_wheel_JointState_.velocity = list(self.bldcMotor.get_angular_velocity())  # rad/s
            msg_wheel_JointState_.position = list(self.bldcMotor.get_wheels_travelled())  # rad
            msg_wheel_JointState_.header.stamp = self.get_clock().now().to_msg()
            self.wheel_JointState_pub_.publish(msg_wheel_JointState_)

        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish JointState: {str(e)}")
        if hasattr(self, 'timer_JointState_'):
            self.timer_JointState_.cancel()

    def sub_Vel_Callback(self, msg):
        """Receive wheel speed commands and send RPM commands to motor"""

        if len(msg.data) != 2:
            return
        # Reconnect logic if motor is disconnected
        if not self.bldcMotor or not self.bldcMotor.is_connected():
            if self.bldcMotor.was_connected:  
                self.get_logger().warn("⚠ Modbus connection lost. Attempting reconnection...")
                self.bldcMotor.was_connected = False  

            self.bldcMotor.reset_motor_after_reconnect()
            if not self.bldcMotor.is_connected():
                self.get_logger().error("❌ Reconnection failed. Command not sent.")
                return  
            self.get_logger().info("✅ Reconnection successful. Resuming control.")
            self.bldcMotor.was_connected = True  
        # Convert RPS to RPM
        leftWheel = msg.data[0] * (60 / (2 * np.pi))
        rightWheel = msg.data[1] * (60 / (2 * np.pi))
        epsilon = 1e-3
        if abs(msg.data[0]) < epsilon and abs(msg.data[1]) < epsilon:
            self.get_logger().info("🛑 Received zero velocities. Stopping motor.")
            leftWheel = 0;  rightWheel = 0;
            self.motor_states = "STOPPED"
            try:
                self.bldcMotor.set_rpm(leftWheel, rightWheel)
            except Exception as e:
                self.get_logger().error(f"❌ Failed to stop motor: {str(e)}")

        # Ignore small velocities to prevent jitter
        leftWheel = self.param.set_stop_rpm if abs(leftWheel) < self.param.ignore_small_speed_threshold else leftWheel
        rightWheel = self.param.set_stop_rpm if abs(rightWheel) < self.param.ignore_small_speed_threshold else rightWheel
        try:
            self.bldcMotor.set_rpm(int(-leftWheel), int(rightWheel))
            self.motor_states = "ACTIVATE"
        except Exception as e:
            self.get_logger().error(f"❌ Error while setting RPM: {str(e)}")
    def exitBLDCMotor(self):
        """Stop the motor safely when ROS shuts down"""
        if self.bldcMotor:
            try:
                print("🛑 Stopping motor immediately!")
                self.bldcMotor.set_rpm(self.param.set_stop_rpm, self.param.set_stop_rpm)  
                self.bldcMotor.disable_motor()
                self.bldcMotor.close_connect()
            except Exception as e:
               print(f"❌ Failed to stop motor: {str(e)}")

    def ResetPos_Callback(self, request, response):
        """Reset the wheel encoder feedback position"""

        if not self.bldcMotor or not self.bldcMotor.is_connected():
            response.success = False
            response.message = '❌ Motor not connected or error occurred.'
            return response
        try:
            success = self.bldcMotor.clear_position(self.param.set_clear_wheel_encoders)
            if success:
                response.success = True
                response.message = '✅ Encoder position reset successful.'
            else:
                response.success = False
                response.message = '❌ Failed to reset encoder. Check connection!'
        except Exception as e:
            response.success = False
            response.message = f'❌ Expection while reseting encoder: {str(e)}'
        return response


def main():
    rclpy.init()
    node = Zlac_Interfaces()
    executor = MultiThreadedExecutor(num_threads=os.cpu_count()) 
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("🛑 Ctrl+C detected! Shutting down Zlac_Interfaces node ...")
    finally:
        node.exitBLDCMotor()
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Zlac_Interfaces node shutdown complete")


if __name__ == '__main__':
    main()
