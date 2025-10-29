#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time 
import numpy as np
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import csv
from visualization_msgs.msg import Marker, MarkerArray
from typing import List, Tuple
from std_msgs.msg import Bool 
import math
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        nhatbot_controller_share = get_package_share_directory('nhatbot_controller')
        waypoint_path = os.path.join(nhatbot_controller_share, 'waypoints', 'f1tenth_waypoint_path.csv')
        
        # Declare parameters
        self.declare_parameter("waypoint_path", "nhatbot_controller.csv") # change when needed
        self.declare_parameter("car_rearframe", "base_link")
        self.declare_parameter("global_frame","map")
        self.declare_parameter("drive_topic", "/cmd_vel")  
        self.declare_parameter("current_waypoint_topic", "/current_waypoint")
        self.declare_parameter("lookahead_waypoint_topic", "/lookahead_waypoint")  
        self.declare_parameter("timer_duration", 10)
        self.declare_parameter("steering_angle_limit", 1.57) #rad
        self.declare_parameter("look_ahead_dist", 1.0) # meter   
        self.declare_parameter("min_lookahead_dist", 0.5)  # meters
        self.declare_parameter("max_lookahead_dist", 1.0)  # meter        
        self.declare_parameter("velocity_percentage", 0.5) # 50% of the max speed of robot

        self.waypoint_path = self.get_parameter("waypoint_path").get_parameter_value().string_value
        self.car_frame     = self.get_parameter("car_rearframe").get_parameter_value().string_value
        self.map_frame     = self.get_parameter("global_frame").get_parameter_value().string_value
        self.drive_topic   = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.current_waypoint_topic   = self.get_parameter("current_waypoint_topic").get_parameter_value().string_value
        self.lookahead_waypoint_topic = self.get_parameter("lookahead_waypoint_topic").get_parameter_value().string_value
        
        self.timer_duration = self.get_parameter("timer_duration").get_parameter_value().double_value
        self.velocity_percent   = self.get_parameter("velocity_percentage").get_parameter_value().double_value

        
        self.x_car_world = None
        self.y_car_world = None


        self.subscription = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)

        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints_visualization', 10)

        self.goal_poses = self.load_waypoint(waypoint_path)
        self.get_logger().info(f"Finished Loading total {len(self.goal_poses)} waypoints from {waypoint_path}")
       
        if self.goal_poses.size == 0:
            self.get_logger().info(f"Waypoints are empty")
            return
        

        self.get_logger().info(f"Publishing Waypoint. Vissualize it on RViz")
        self.publish_waypoints(self.goal_poses)




    def amcl_pose_callback(self, msg):
        # Extract the position and orientation
        self.x_car_world = msg.pose.pose.position.x
        self.y_car_world = msg.pose.pose.position.y

    def get_closest_point(self,waypoints, lookahead_distance=0.2):

        if len(waypoints) == 0:
            self.get_logger().warn(f"waypoints not available")
            return 
        
        if self.y_car_world is None or self.x_car_world is None:
            self.get_logger().warn(f"x_car_world and y_car_world is None value")
            return
        
        waypoints_X = np.array([w[0] for w in waypoints])
        waypoints_Y = np.array([w[1] for w in waypoints])

        shortest_distance = self.distance_between_2points(waypoints_X[0], waypoints_Y[0],  self.x_car_world, self.y_car_world)
        shortest_i = 0
        for i in range(len(waypoints)):
            dist = self.distance_between_2points(waypoints_X[i], waypoints_Y[i], self.x_car_world,  self.y_car_world)
            if dist <= shortest_distance:
                shortest_distance = dist
                shortest_i = i


    def visualize_lookahead_point(self, point):
        # Create a Marker message      
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg() # Use ROS 2 Clock for the timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
               
        # Set the position of the marker
        marker.pose.position.x = self.goal_poses[point][0]
        marker.pose.position.y = self.goal_poses[point][1]
        marker.pose.position.z = 0.0
        
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Set an ID for the marker
        marker.id = 1

        # Publish the marker to RViz or other visualizers
        self.lookahead_waypoint_pub_.publish(marker)
        # self.get_logger().info("Published lookahead waypoint as a marker")
             
    def visualize_current_point(self, point):
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg() # Use ROS 2 Clock for the timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        # Set the position of the marker
        marker.pose.position.x = self.goal_poses[point][0]
        marker.pose.position.y = self.goal_poses[point][1]
        marker.pose.position.z = 0.0
        
        # Set an ID for the marker
        marker.id = 1

        # Publish the marker to RViz or other visualizers
        self.cur_waypoint_pub_.publish(marker)
        # self.get_logger().info("Published current waypoint as a marker")

    def publish_waypoints(self, list_of_waypoints)->None:
        marker_array = MarkerArray() 
        marker_array.markers.append(Marker(action=Marker.DELETEALL))

        for i, pose in enumerate(list_of_waypoints):

            # Create a marker for each waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0.0  # Assuming a 2D plane
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2  # Size of the sphere
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.a = 1.0
            # Set color based on waypoint index
            if i == 0 or i == 1:
                marker.scale.x = 0.3  # Size of the sphere
                marker.scale.y = 0.3
                marker.scale.z = 0.3

                marker.color.r = 0.0  # Red
                marker.color.g = 0.0  # Green
                marker.color.b = 1.0  # Blue (Green in this case)

            else:
            
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                    
            marker_array.markers.append(marker)
        
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published waypoints as markers")

        
    def load_waypoint(self, file_path:str) -> List[Tuple[float, float, float]]:
        goal_poses = []
        if os.path.exists(file_path):
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  
                for row in reader:
                    if len(row) == 3: 
                        x, y, z = map(float, row)
                        goal_poses.append((x, y, z))
        else:
            self.get_logger().error(f"CSV file not found: {file_path}")
            
        remove_goal_poses  =  self.remove_overlapped_waypoints(goal_poses, threshold= 0.2)
        smoother_goal_poses = self.smoother_waypoint(remove_goal_poses)
        return smoother_goal_poses
    def smoother_waypoint(self, waypoints_list, weight_data = 0.2 , weight_smooth = 0.8, tolerance = 0.0001) -> None: 
        smoother_waypoints_list = np.array(waypoints_list)
        waypoints_list = np.array(waypoints_list)
        
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1,  len(smoother_waypoints_list) - 1):
                aux = np.copy(smoother_waypoints_list[i, :2])  # Store the original values of the first two components
                
                # Apply the smoothing formula only to the first two values (x and y)
                smoother_waypoints_list[i, :2] += weight_data * (waypoints_list[i, :2] - smoother_waypoints_list[i, :2]) + \
                                        weight_smooth * (smoother_waypoints_list[i - 1, :2] + 
                                                        smoother_waypoints_list[i + 1, :2] - 
                                                        2.0 * smoother_waypoints_list[i, :2])
                # Accumulate the total change (sum of absolute differences)
                change += np.sum(np.abs(aux - smoother_waypoints_list[i, :2]))
                    
        return smoother_waypoints_list

    def remove_overlapped_waypoints(self, waypoints_list, threshold = 0.1 ):
        cleaned_waypoints_list = [waypoints_list[0]]  # Start with the first waypoint
    
        for i in range(1, len(waypoints_list)):
            x1, y1,_ = cleaned_waypoints_list[-1]
            x2, y2,_ = waypoints_list[i]        
            if self.distance_between_2points(x1,y1, x2, y2) >= threshold:
                cleaned_waypoints_list.append(waypoints_list[i])
                
        return cleaned_waypoints_list
    
    def distance_between_2points(self,x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt(math.pow((x2 - x1),2)  + math.pow((y2 - y1), 2))
 

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass
        # pure_pursuit_node.drive_msgObj.drive.speed = 0.0
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()