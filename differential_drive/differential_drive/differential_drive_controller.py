#!/usr/bin/env python3

import numpy as np
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped, Twist ,TwistStamped

from sensor_msgs.msg import  JointState 
from rclpy.time import Time 
from rclpy.constants import S_TO_NS

from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger


class Differential_Drive_Controller(Node):
    def __init__(self):
        super().__init__("differential_drive_controller_node")

        self.declare_parameter("wheel_radius", 0.0535); self.declare_parameter("wheel_separation", 0.45)
        
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value 

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius_); self.get_logger().info("Using wheel_separation %f" % self.wheel_separation_)


        self.max_linear_velocity = 0.6  # m/s (tốc độ tiến/lùi tối đa)
        self.min_linear_velocity = -0.6  # m/s (tốc độ tiến/lùi tối thiểu)
        self.max_angular_velocity = 0.4  # rad/s (tốc độ quay tối đa)
        self.min_angular_velocity = -0.4  # rad/s (tốc độ quay tối thiểu)


        self.left_wheel_prev_pos_ = 0.0;  self.right_wheel_prev_pos_ = 0.0
       
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0; self.y_ = 0.0; self.theta_ = 0.0

        self.phi_left = 0.0; self.phi_right = 0.0

        self.dp_left = 0; self.dp_right = 0;
        

        self.group_Reent_ = ReentrantCallbackGroup()

       # SUBSCRIBER

        self.vel_sub_ = self.create_subscription(Twist,"nhatbot_controller/cmd_vel", self.velCallback, 10, callback_group=self.group_Reent_) # Subscribe to the velocities commands that are coming from the joystick

        self.joint_sub_ = self.create_subscription(JointState, "nhatbot_firmware/JointState", self.jointStateCallback,10, callback_group=self.group_Reent_)

                                            
        # PUBLISHER

        self.wheel_cmd_pub_ = self.create_publisher(Float32MultiArray, "nhatbot_controller/wheel_rotational_vel", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "nhatbot_controller/odom", 10)
        self.motor_srv_ = self.create_service(Trigger, "reset_odom", self.ResetOdom_Callback)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2.0, self.wheel_radius_/2.0], 
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        

                                        # Initialize ODOM variables


###############################################################################

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"


    # def velCallback(self, msg):
    #     """
    #     This methods aims to subscribe the velocities from joystick and convert these values to rpm and published to the robot
    #     """
     
    #     wheel_speed_msg = Float32MultiArray()
        
    #     robot_velocities = np.array([[msg.linear.x],
    #                                 [msg.angular.z]])
        
    #     wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_velocities)

    #     wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]

        # right_wheel = msg.linear.x + ((msg.angular.z * self.wheel_separation_) / 2.);  
        # left_wheel = msg.linear.x - ((msg.angular.z * self.wheel_separation_) / 2.); 


    #     wheel_speed_msg.data = [left_wheel/self.wheel_radius_, right_wheel/self.wheel_radius_]
        
    #     self.wheel_cmd_pub_.publish(wheel_speed_msg)
    def velCallback(self, msg):
        """
        This method subscribes to the velocities from the joystick,
        limits the velocities, converts them to wheel RPMs,
        and publishes the RPMs to the robot.
        """

        # 1. Giới hạn tốc độ đầu vào
        linear_vel = np.clip(msg.linear.x, -0.6, 0.6)      # m/s
        angular_vel = np.clip(msg.angular.z, -0.4, 0.4)    # rad/s

        # 2. Tính vận tốc từng bánh (m/s)
        right_wheel = linear_vel + (angular_vel * self.wheel_separation_ / 2.0)
        left_wheel = linear_vel - (angular_vel * self.wheel_separation_ / 2.0)

        # 3. Chuyển vận tốc từ m/s sang rad/s: v = ω * r → ω = v / r
        right_rads = right_wheel / self.wheel_radius_
        left_rads = left_wheel / self.wheel_radius_

        # 4. (Tuỳ chọn) Nếu bạn cần chuyển từ rad/s sang RPM:
        right_rpm = right_rads * 60 / (2 * np.pi)
        left_rpm = left_rads * 60 / (2 * np.pi)

        # 5. Tạo và xuất thông điệp
        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [left_rpm, right_rpm]  # Left - Right theo thứ tự

        # 6. Gửi lệnh đến động cơ
        self.wheel_cmd_pub_.publish(wheel_speed_msg)



    def jointStateCallback(self, msg):

        self.dp_left = msg.position[0] - self.left_wheel_prev_pos_  ;   self.dp_right = msg.position[1] - self.right_wheel_prev_pos_

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ 

        #self.get_logger().info("dt: %s" % (dt))

        self.left_wheel_prev_pos_ = msg.position[0] ; self.right_wheel_prev_pos_ = msg.position[1] ;

        self.prev_time_ = Time.from_msg(msg.header.stamp) 


        self.phi_left = self.dp_left / (dt.nanoseconds / S_TO_NS); self.phi_right = self.dp_right / (dt.nanoseconds / S_TO_NS)
        
       # self.get_logger().info("phi_left: %f, phi_right: %f" % (self.phi_left, self.phi_right))

        linear = (self.wheel_radius_ * self.phi_right + self.wheel_radius_ * self.phi_left) / 2.

        angular = (self.wheel_radius_ * self.phi_right - self.wheel_radius_ * self.phi_left) / self.wheel_separation_

        # self.get_logger().info("linear: %f, angular: %f" % (linear, angular))


        d_s = (self.wheel_radius_ * self.dp_right + self.wheel_radius_ * self.dp_left) / 2.
        d_theta = (self.wheel_radius_ * self.dp_right - self.wheel_radius_ *  self.dp_left) / self.wheel_separation_

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        #self.get_logger().info("x: %f, y: %f, theta: %f" % (self.x_, self.y_, self.theta_))
        # if(self.theta_ > np.pi): self.theta_ -= 2*np.pi
        # elif (self.theta_ < np.pi): self.theta_ += 2*np.pi

        #self.get_logger().info("x: %f, y: %f, theta: %f" % (self.x_, self.y_, self.theta_))


        q = quaternion_from_euler(0, 0 , -self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        
        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.transform_stamped_)


    def ResetOdom_Callback(self,request, response):
        self.dp_left = 0; self.dp_right = 0; self.theta_ = 0; self.x_ = 0; self.y_ = 0; 
        
        response.success = True
        response.message = 'Odom has been reset'
        return response

def main():
    try:
        rclpy.init()
        simple_controller_node = Differential_Drive_Controller()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(simple_controller_node)
        try:
            executor.spin()
        finally: 
            executor.shutdown()
            simple_controller_node.destroy_node()

    except KeyboardInterrupt:
        simple_controller_node.get_logger().info('Ctrl+C received - exiting...')
        #sys.exit(0)
    except ExternalShutdownException:
        sys.exit(1)
    
    finally:
        simple_controller_node.get_logger().info('controller node is shutdowning')

        rclpy.shutdown()


if __name__ == '__main__':
    main()