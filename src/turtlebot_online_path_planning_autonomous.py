#!/usr/bin/python3

import numpy as np
import rospy
import tf 
import time
import math

from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion

from utils_lib.online_planning_autonomous import *   
#from utils_lib.editing_rrt_star import RRTAlgorithm  
class OnlinePlanner:
 
    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, dominion, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        self.env = None 
        self.maz = 0 
        self.stop_exploration = False 
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Current speed of the robot
        self.current_v = 0.0
        # current angular velocity
        self.current_w = 0.0
        # Define imu angle
        self.Imu_angle = [0, 0, 0]
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.dominion = dominion                                        

        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 0.5
        # Proportional angular velocity controller gain                   
        self.Kw = 0.5
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.3  
        # Define target velocity
        self.target_velocity = 0.5    
        self.aruco_pose = 0
        
        self.wheel_radius = 0.035 # meters      
        self.wheel_base_distance = 0.257 # meters  
        self.d = distance_threshold         

        # PUBLISHERS
        # Publisher for sending velocity commands to the robot  
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Float64MultiArray, queue_size=10)   
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        #Publisher for wiggly path
        self.wiggly_pub = rospy.Publisher('~wiggly_marker', Marker, queue_size=1)  
        
        #Publisher for asking for goal position
        self.goal_reached_pub = rospy.Publisher('goal_reached', Bool, queue_size=10)
        
        #robot square publisher
        self.square_pub = rospy.Publisher('square', Marker, queue_size=10)
        # SUBSCRIBERS
        # self.gridmap_sub = None # TODO: subscriber to gridmap_topic from Octomap Server   
        self.image_sub = rospy.Subscriber("/small_aruco_position", Float64MultiArray, self.aruco_subscriber) 
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)  
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom) 
        self.imu_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/imu_data", Imu, self.Imu_callback)   
        self.move_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)     
         
        # TIMERS  
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.06), self.controller)

    def aruco_subscriber(self, msg):
            print("Object detected.... stopping exploration ....")
            self.stop_exploration = True
            self.aruco_pose = msg.data
            self.aruco_pose = self.aruco_pose[0:2]   
            self.goal = self.aruco_pose  
            self.__send_commnd__(0, 0)    
            self.path = self.plan()  

    def Imu_callback(self, Imu_msg): 
        self.imu_vel = Imu_msg.angular_velocity.z
        self.imu_quaternion = (Imu_msg.orientation.x, Imu_msg.orientation.y, Imu_msg.orientation.z, Imu_msg.orientation.w) 
        self.Imu_angle = euler_from_quaternion(self.imu_quaternion)  
    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, wrap_angle(yaw)])
        self.current_v = odom.twist.twist.linear.x
        self.current_w = odom.twist.twist.angular.z
        
        marker = Marker()
        marker.header.frame_id = "world"  
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.0) 


        marker.id = 444

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = self.current_pose[0]
        marker.pose.position.y = self.current_pose[1]
        marker.pose.position.z = -0.1 
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1 

        marker.color.r = 1.0
        marker.color.a = 0.9
        
        vertices = [(self.d, self.d), (-self.d, self.d), (-self.d, -self.d), (self.d, -self.d), (self.d, self.d)]
        
        for vertex in vertices:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            
            marker.points.append(point)
            
        self.square_pub.publish(marker)
            
    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method 
    def get_goal(self, goal):
        # if self.stop_exploration == False: 
            self.maz = 0
            rospy.set_param('goal_pose_turtle', 0)   
            if self.svc.there_is_map:
                print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
                self.goal = np.array([goal.pose.position.x, goal.pose.position.y]) 
                
                print('goal',self.goal)   
                if self.svc.is_valid(self.goal) == True:   #check if the goal is reachable
                    self.__send_commnd__(0, 0)  
                    self.path = []                                                   
                    self.path = self.plan()
                    self.goal_reached = False
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = self.goal_reached

                # publish the message
                    self.goal_reached_pub.publish(goal_reached_msg)
                else:
                    print ("the given goal is in an occupied position") 
                    
                    self.goal_reached = True
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = self.goal_reached

                # publish the message
                    self.goal_reached_pub.publish(goal_reached_msg)
            
        
    # Map callback:  Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
      
        # to avoid map update too often (change value if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            self.env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set( self.env, gridmap.info.resolution, origin) 

            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0: 
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path
                # TODO: check total_path validity. If total_path is not valid make self.path = None and replan
                if not self.svc.check_path(total_path, 0.1):  
                    self.path = []
                    self.goal_reached = True
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = self.goal_reached

                # publish the message
                    self.goal_reached_pub.publish(goal_reached_msg)
                    self.__send_commnd__(0, 0) 
                    self.path = self.plan() 
                    
    


    # Solve plan from current position to self.goal. 
    def plan(self):
        # List of waypoints [x, y] that compose the plan
        path = []
        trial = 0
        # If planning fials, allow replanning for several trials 
        while len(path) == 0 and trial < 10:
            print("Compute new path") 
            #print(self.current_pose)
            # TODO: plan a path from self.current_pose to self.goal
            while not self.svc.is_valid(self.current_pose[0:2]):
                obstacle = self.svc.get_closest_obstacle(self.current_pose[0:2])
                angle = get_angle(self.current_pose[0:2],obstacle)
                print("angle", math.degrees(angle))
                 # Calculate the difference between robot theta and the angle to closest obstacle
                angle_difference = self.current_pose[2] - angle
                print("angle difference", math.degrees(angle_difference))   
                # Check if the absolute angle difference is less than pi/2 (90 degrees)
                if abs(angle_difference) <= math.pi / 2:
                    print("the obstacle is in front")
                    start_time = time.time()
                    while time.time() - start_time < 0.5: # 1.0
                        self.__send_commnd__(-0.15, 0.0)
                    self.__send_commnd__(0.0, 0.0)
                else:
                    print("the obstacle is behind")
                    start_time = time.time()
                    while time.time() - start_time < 0.5:  # 1.0  
                        self.__send_commnd__(0.15, 0.0)
                    self.__send_commnd__(0.0, 0.0)
                
            path = compute_path(self.current_pose[0:2], self.goal, self.svc, self.dominion, 4.0)   
            
            
            if path == []:
                self.goal_reached = True
                goal_reached_msg = Bool()
                goal_reached_msg.data = self.goal_reached

            # publish the message
                self.goal_reached_pub.publish(goal_reached_msg)
            self.publish_path(path,3)
            
            path = self.smooth_path(path)   
            print('path', path) 
            # calculate the difference in x and y coordinates
            delta_x = self.current_pose[0] - self.goal[0] 
            delta_y = self.current_pose[1] - self.goal[1]

            # calculate the angle in radians using atan2
            angle_rad = wrap_angle(math.atan2(delta_y, delta_x) ) 
            final_orientation = angle_rad  + np.pi

            # if you don't want dubins path, comment five lines below 

            # path = point_to_pose(path, self.current_pose[2], final_orientation) 
            # turning_radius = 0.08 
            # step_size = 0.1
            # path = dubins_maz(turning_radius, step_size, path)                              
            # print('dubins_path', path)    
            
            trial += 1
        if trial == 10:
            # If planning fails, consider increasing the planning time
            print("Path not found!") 
            self.goal_reached = True
            goal_reached_msg = Bool()
            goal_reached_msg.data = self.goal_reached

            # publish the message
            self.goal_reached_pub.publish(goal_reached_msg)
        else:
            print("Path found")
            self.goal_reached = False
            goal_reached_msg = Bool()
            goal_reached_msg.data = self.goal_reached

            # publish the message
            self.goal_reached_pub.publish(goal_reached_msg)
            # Publish plan marker to visualize in rviz
            self.publish_path(path, 1) 
            # remove initial waypoint in the path (current pose is already reached)
            del path[0]                 
        return path 


    def smooth_path(self, path): 
        path.reverse()
        first_point = path[0]
        next_point = path[1] 
        new_path = [first_point]
        for point in range(1, len(path)):
            if self.svc.check_path([first_point, path[point]], 0.15):
                next_point = path[point]
            else:
                new_path.append(next_point) 
                first_point = next_point 
        
        new_path.append(path[-1])
        new_path.reverse()
        return new_path

    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.
    def controller(self, event):
        v = 0
        w = 0
        if self.path is not None and len(self.path) > 0:
            
            if len(self.path) > 1:
                if np.linalg.norm(self.path[0][0:2] - self.current_pose[0:2]) < 2*self.svc.resolution and np.linalg.norm(self.path[1][0:1] != self.goal):
                    print("Position {} reached".format(self.path[0]))
                    print('cuurent orientation', self.current_pose[2])  
                    del self.path[0] 
                   
                else:
                    # TODO: Compute velocities using controller function in utils_lib
                    
                    v, w = move_to_point(self.current_pose, self.path[0][0:2], self.Imu_angle)  
                    
                    self.goal_reached = False 
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = self.goal_reached 

                    # publish the message
                    self.goal_reached_pub.publish(goal_reached_msg)
                    self.__send_commnd__(v, w) 
            else:
                if np.linalg.norm(self.path[0][0:2] - self.current_pose[0:2]) > 5*self.svc.resolution: 
                    v, w = move_to_point(self.current_pose, self.path[0][0:2], self.Imu_angle)  
                    self.__send_commnd__(v, w)  
                else: 
                    self.path = None 
                    rospy.set_param('goal', 1)  
                    v=0
                    w=0
                    print("Final position reached!")
                    self.goal_reached = True
                    goal_reached_msg = Bool() 
                    goal_reached_msg.data = self.goal_reached
 
                    v=0
                    w=0
                    print("Final position reached!") 
                    self.goal_reached = True
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = self.goal_reached
                    
                    # publish the message 
                    self.goal_reached_pub.publish(goal_reached_msg) 
                    
                    self.__send_commnd__(v, w) 
                    if  self.stop_exploration == True:
                        rospy.set_param('goal_pose_turtle', 1)  
                        print('object is detected and stopping exploration') 
                        

    # PUBLISHER HELPERS

    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):

        rate = rospy.Rate(100)   
        move = Float64MultiArray() 
       
         
        v_l = (2 * v + w * self.wheel_base_distance) / (2 * self.wheel_radius)
        v_r = (2 * v - w * self.wheel_base_distance) / (2 * self.wheel_radius) 

        move.data = [v_l, v_r]         
   
        self.cmd_pub.publish(move) 
         

    # Publish a path as a series of line markers
    def publish_path(self, path, id): 
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            if id == 1:
                color_red = ColorRGBA()
                color_red.r = 1
                color_red.g = 0
                color_red.b = 0
                color_red.a = 1
                color_blue = ColorRGBA()
                color_blue.r = 0
                color_blue.g = 0
                color_blue.b = 1
                color_blue.a = 1
            else:
                color_red = ColorRGBA()
                color_red.r = 0
                color_red.g = 1
                color_red.b = 0
                color_red.a = 1
                color_blue = ColorRGBA()
                color_blue.r = 0
                color_blue.g = 1
                color_blue.b = 0
                color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            if id == 1:
                self.marker_pub.publish(m)
            else:
                self.wiggly_pub.publish(m)
            
# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')   
    node = OnlinePlanner('/projected_map', 'kobuki/odom', '/turtlebot/kobuki/commands/wheel_velocities', np.array([-15.0, 15.0]), 0.25)   
    
    # Run forever
    rospy.spin()