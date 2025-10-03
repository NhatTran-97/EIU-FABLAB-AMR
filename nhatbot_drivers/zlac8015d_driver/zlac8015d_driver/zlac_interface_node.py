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
import time

# Main motor interface node
class ZlacInterfaces(Node):
    def __init__(self, ):
        super().__init__('zlac_driver_node')
        self._shutting_down = False

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
                                                                 self.sub_vel_callback, qos_cmd , callback_group=self.ReentGroup)
            self.timer_JointState_ = self.create_timer(self.param.joint_state_frequency, self.pub_jointstate_callback, callback_group=self.ReentGroup)
            self.timer_zlacStatus_ = self.create_timer(self.param.zlac_status_frequency, self.zlac_status_callback, callback_group=self.ReentGroup)
            self.motor_srv_ = self.create_service(Trigger, self.param.reset_encoder_service, self.reset_pos_callback)
        
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


    def zlac_status_callback(self):
        """
        Periodically publishes the current ZLAC8015D motor status as a ZlacStatus message.
        Includes battery voltage and (optionally) other motor diagnostics.
        """
        if self._shutting_down or not rclpy.ok():
            return
        try:
   
            zlac_status = ZlacStatus()
            zlac_status.battery_voltage = float(self.bldcMotor.get_battery_voltage())
            zlac_status.brake_state = str(self.bldcMotor.get_brake_state())
            zlac_status.control_mode = int(self.bldcMotor.get_mode())
            zlac_status.driver_temp = float(self.bldcMotor.get_driver_temperature())
            zlac_status.vehicle_state = str(self.motor_states)
            try:
                self.zlac_status_pub_.publish(zlac_status)
            except Exception as e:
                if not self._shutting_down and rclpy.ok():
                    self.get_logger().error(f"❌ Failed to publish ZlacStatus: {e}")
        except Exception as e:
            if not self._shutting_down and rclpy.ok():
                self.get_logger().error(f"❌ ZlacStatus error: {e}")


    def pub_jointstate_callback(self):
        if self._shutting_down or not rclpy.ok():
            return
        if not self.bldcMotor or not self.bldcMotor.is_connected():
            return

        try:
            vel = self.bldcMotor.get_angular_velocity()    
            pos = self.bldcMotor.get_wheels_travelled()    

            if (not isinstance(vel, (list, tuple)) or not isinstance(pos, (list, tuple)) or len(vel) != 2 or len(pos) != 2 or any(v is None for v in vel) or any(p is None for p in pos)):
                if not self._shutting_down and rclpy.ok():
                    print("⚠ JointState skipped (invalid data).")
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.velocity = [float(vel[0]), float(vel[1])]
            msg.position = [float(pos[0]), float(pos[1])]

            try:
                self.wheel_JointState_pub_.publish(msg)
            except Exception as e:
                
                if not self._shutting_down and rclpy.ok():
                    self.get_logger().error(f"❌ Failed to publish JointState: {e}")
        except Exception as e:
            if not self._shutting_down and rclpy.ok():
                self.get_logger().error(f"❌ Failed to publish JointState: {e}")

    def sub_vel_callback(self, msg):
        """Receive wheel speed commands and send RPM commands to motor"""

        if self._shutting_down or not rclpy.ok():
            return
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

    def reset_pos_callback(self, request, response):
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


    def prepare_shutdown(self):
        self._shutting_down = True

        if hasattr(self, 'timer_JointState_') and self.timer_JointState_:
            try:
                self.timer_JointState_.cancel()
                self.destroy_timer(self.timer_JointState_)
            except Exception:
                pass
            self.timer_JointState_ = None

        if hasattr(self, 'timer_zlacStatus_') and self.timer_zlacStatus_:
            try:
                self.timer_zlacStatus_.cancel()
                self.destroy_timer(self.timer_zlacStatus_)
            except Exception:
                pass
            self.timer_zlacStatus_ = None
        time.sleep(0.05)
        self.exitBLDCMotor()

        if hasattr(self, 'wheel_JointState_pub_') and self.wheel_JointState_pub_:
            try: self.destroy_publisher(self.wheel_JointState_pub_)
            except Exception: pass
            self.wheel_JointState_pub_ = None

        if hasattr(self, 'zlac_status_pub_') and self.zlac_status_pub_:
            try: self.destroy_publisher(self.zlac_status_pub_)
            except Exception: pass
            self.zlac_status_pub_ = None


def main():
    rclpy.init()
    node = ZlacInterfaces()
    executor = MultiThreadedExecutor(num_threads=os.cpu_count()) 
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("🛑 Ctrl+C detected! Shutting down Zlac_Interfaces node ...")
    finally:
        node.prepare_shutdown()        
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Zlac_Interfaces node shutdown complete")


if __name__ == '__main__':
    main()
