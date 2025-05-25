#!/usr/bin/env python3

import math 
import time 
import numpy as np 
import rclpy
from rclpy.node import Node
import rclpy.qos
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan, Joy, JoyFeedbackArray, JoyFeedback 
from ackermann_msgs.msg import AckermannDriveStamped 
from nav2_msgs.msg import Odometry


class EmergencyStop(Node):
    """Node for handling emergency stops."""
    def __init__(self):
        """Initialize the EmergencyStop node"""
        super().__init__('EmergencyStop')

        self.ranges = None 
        self.angle_min = None 
        self.angle_max = None 
        self.angle_increment = None 
        self.range_min = None 
        self.range_max = None

        # cosine values for each angle in the laser scan 
        self.cosines = []

        # distance from the car to the nearest obstacle in each direction (from left to right)
        self.car_distance = []

        # create a publisher for the /emergency_Stop_topic 
        self.publishers_ = self.create_publisher(
            AckermannDriveStamped, 'emergency_Stop_topic', 10)
        self.subscription_ = self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.subscription_
        self.velocity = 0
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth = 1,
                                   reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, qos)

        self.emergency_active = True 
        self.last_button_state = 0 # The previous state of the button

        # Load parameters from the parameter server
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ttc_threshold1', 1.0), # 0.45 this should be changed after testing
                ('ttc_threshold2', 1.0), # 0.35
                ('ttc_threshold3', 1.0), # 0.06
                ('wheelbase', 0.3302),
                ('width', 0.2032),
                ('scan_to_base_link', 0.275),
                ('scan_field_of_view', 6.2831853),
                ('scan_beams', 1080),
                ('mode', "normal") # default value, can be overriden by launch file
            ]
        )


        # set FOV for each region in degrees
        self.index_list_con = []
        self.first_region_fov = 15
        self.second_region_fov = 30
        self.third_region_fov = 270

        self.set_default_index_values() # set the default index region values
        self.create_index_list_con() #

        # Get the TTC threshold parameter value
        self.ttc_threshold1 = self.get_parameter('ttc_threshold1').value
        self.ttc_threshold2 = self.get_parameter('ttc_threshold2').value
        self.ttc_threshold3 = self.get_parameter('ttc_threshold3').value

        self.wheelbase = self.get_parameter('wheelbase').value
        self.width = self.get_parameter('width').value
        self.scan_distance_to_base_link = self.get_parameter('scan_to_base_link').value
        self.scan_fov = self.get_parameter('scan_field_of_view').value
        self.scan_beams = self.get_parameter('scan_beams').value
        self.scan_ang_inc = self.scan_fov / self.scan_beams
        self.mode = self.get_parameter('mode').value  # 'normal mode' or 'adaptive mode'

        self.steering_angle = 0.0 # rad - straight ahead

        if self.mode == 'adaptive':
            self.adjusted = False
            self.subscription = self.create_subscription(AckermannDriveStamped, 'ackermann_cmd', self.ackermann_cmd_callback, 10)
    

    def get_cosines(self, scan_beams, angle_min, angle_increment):
        """Calculate the cosine values for each angle in the laser scan"""
        for i in range (scan_beams):
            angle = angle_min + i*angle_increment
            self.cosines.append(math.cos(angle))
    
    def get_car_distance(self, width, wheelbase, scan_distance_to_base_link, angle_min, angle_max, angle_increment):
        """Calculate the distance from the car to the nearest obstacle in each direction"""
        # calculate the distance from the car to the nearest obstacle
        # retrun the distance
        # Hint: use self.ranges anf self.cosines

        self.car_distance = []
        dist_to_side = width / 2 # distance from the side of the car to the laser scan
        dist_to_front = wheelbase - scan_distance_to_base_link # distance from the front of the car to the laser scan
        dist_to_back = scan_distance_to_base_link # distance from the back of the car to the laser scan

        for i in range(len(self.ranges)):
            angle = angle_min + i*angle_increment
            if angle > 0:
                if angle < math.pi/2:
                    # between 0 and pi/2
                    to_side = dist_to_side / math.cos(angle - math.pi/2)
                    to_back = dist_to_back / math.sin(angle - math.pi/2)
                    self.car_distance.append(min(to_side, to_back))

    
