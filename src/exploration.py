#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import rospy
import tf
import random
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped , PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from utils_lib.online_planning import *

from std_msgs.msg import ColorRGBA, Float64MultiArray



import math

class exploration:
    def __init__(self):
        self.last_map_time = rospy.Time.now()
        self.origin = []
        self.map = []
        self.map_size = []
        self.current_pose = []
        self.goal = None
        self.resolution = None
        self.goal_reached = True
        self.svc = StateValidityChecker()
        self.rectangle = 0.25 
        self.stop_exploration = False
        
        # Define Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.frontier_pub = rospy.Publisher('/frontier', PoseArray, queue_size=10)
        self.cluster_pub = rospy.Publisher('/cluster', MarkerArray, queue_size=10)
        
        # Define subscribers
        self.image_sub = rospy.Subscriber("/small_aruco_position", Float64MultiArray, self.aruco_subscriber)
        self.odom_sub = rospy.Subscriber('kobuki/odom', Odometry, self.get_odom)  
        self.gridmap_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.get_gridmap)
        self.reached_sub = rospy.Subscriber('goal_reached', Bool, self.reached)

    def aruco_subscriber(self, msg):
            print("Object detected.... stopping exploration ....")
            self.stop_exploration = True
            rospy.signal_shutdown("ArUco marker detected") 
         
    def angle(self, position1, position2):
        x1 = position1[0]
        x2 = position2[0]
        y1 = position1[1] 
        y2 = position2[1]
        return wrap_angle(math.atan2(y2 - y1, x2 - x1))  
      
    def reached(self, msg):
        """Callback function to communicate with the other node to know weather a new goal is needed or not

        Args:
            msg (callback message of type boolean): says if the goal is reached or not
        """
        if msg.data == False:
            self.goal_reached = False
        else:
            self.goal_reached = True
       
        

    def get_gridmap(self,gridmap):
        """Callback function that is executed everytime the map is updated

        Args:
            gridmap (callback msg of type OccupancyGrid): Grid map
        """
        
        # to avoid map update too often (change value if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            self.origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.resolution = gridmap.info.resolution
            self.map = env
            
            
            self.map_size = self.map.shape
    
            if self.stop_exploration == False: 
                # Extract frontier points using BFS
                frontier_points = self.bfs_frontier(self.map)
                
                if self.goal_reached:
                    self.frontier_publisher(frontier_points)
                    # Call bfs_frontier_segment function
                    segment_points = self.bfs_frontier_segment(frontier_points)
                    marker_array = MarkerArray()
                    x=1
                    for i in segment_points:   
                        marker = self.Cluster_publisher(i, x, (random.random(), random.random(), random.random()))
                        marker_array.markers.append(marker)
                        x=x+1
                    self.cluster_pub.publish(marker_array)
                    point_to_go = self.explore(segment_points)
                    if point_to_go != None:
                        self.goal_pubisher(point_to_go)
                
                
                
            
    


    def explore(self, segment_points):
        """This function is the main exloration function. First it computes the frontier segments length and select the longest 
        and them compute the distance between the robot current position and the points in the selected segment and select the closest to the average distances
        Finally it checks if the selected position is valid and returns it. If its not valid it selects the next one in the list and checks again


        Args:
            segment_points (list of list of points): list of clustered frontiers
        """
        var1=True
        while var1 and segment_points !=[]:
            
            # Create a list called number that has the number of segments as size
            number = np.zeros((1,len(segment_points)))
            number = number[0] 
            dist = np.zeros((1,len(segment_points)))
            dist = dist[0] 
            angle = np.zeros((1,len(segment_points)))
            angle = angle[0]
            index1=0 
            
            #Calculate the length of every segment and save it in the list "number"
            for i in segment_points:
                number[index1] = len(i)
                index1=index1+1
            index3=0
            for i in segment_points:
                min_dist = np.inf
                for j in i:
                    if self.distance(j, self.current_pose) < min_dist:
                        dist[index3] = self.distance(j, self.current_pose)
                index3 = index3+1
            
            index4=0 
            for i in segment_points:
                #min_angle = np.inf
                # Create a list called distances that has the number of points in the segment as size
                distances1 = np.zeros((1,len(i)))
                distances1 = distances1[0]
                index5=0
                
                # Calculate the distance between the current position of the robot and each of the points in the segment and save the value in the distances list
                for j in i:
                    distances1[index5]=self.distance(j,self.current_pose)
                    index5=index5+1
                    
                # Calculate the average between the distances and select the closest distance to the average as the goal.
                avgerage = distances1.mean()
                differance = np.abs(distances1 - avgerage)
                min_ind = np.argmin(differance)           
                points_to_compare = i[min_ind]
                # angle[index4] = np.abs(self.angle(points_to_compare, self.current_pose))
                index4 = index4+1
            combine = np.zeros((1,len(segment_points)))
            combine = combine[0] 
            for i in range(len(segment_points)):
                combine[i]= (number[i]/100)+ 10/(dist[i]) + 2/(angle[i])
            print("combine = ",combine)
            print("number = ",number)
            print("dist = ",dist)
            
            # Get the longest segment and select it as the next target   
            max_index = np.argmax(combine) 
            print("max", max_index)
            segment_to_go = segment_points[max_index]
            
            
            
            var2 =True
            while var2 and segment_to_go != []:
                # Create a list called distances that has the number of points in the segment as size 
                distances = np.zeros((1,len(segment_to_go)))
                distances = distances[0]
                index2=0
                
                # Calculate the distance between the current position of the robot and each of the points in the segment and save the value in the distances list
                for i in segment_to_go:
                    distances[index2]=self.distance(i,self.current_pose)
                    index2=index2+1
                    
                # Calculate the average between the distances and select the closest distance to the average as the goal.
                avg = distances.mean()
                diff = np.abs(distances - avg)
                min_index = np.argmin(diff)           
                j = segment_to_go[min_index]
                point_to_go = self.map_to_position(j)
                self.goal = point_to_go
                
                # Check if the goal is valid, if not, remove it from the segment and consider the next point
                if self.is_valid(self.goal) == False:
                    segment_to_go.remove(j)
                else:
                    var2=False
                    
                    
            # if all the points are not valid in the segment, remove the segment and consider the next one        
            if var2==True:
                segment_points.remove(segment_points[max_index])
            else:
                var1=False
                
        # Publish the goal if there are still frontiers to discover   
        if not segment_points:
            print("map is explored")
            return None
            
        else:
            return point_to_go
            
         
            
            
    def get_odom(self, odom):
        """Callback function that subscribes to the odometry of the robot

        Args:
            odom (callback message of type Odometry): Get the odometry of the robot
        """
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        
    def distance (self, current, goal):
        """This function calculates the distance between 2 given points

        Args:
            current (tuple): point1
            goal (tuple): point2

        Returns:
            float: distance between point1 and point2
        """
    
        return math.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    
    def is_valid(self, pose):
        """Given a pose, returs true if the pose is not in collision and false othewise.

        Args:
            pose (tuple): position

        Returns:
            boolean: Valid or not
        """
        coord = self.__position_to_map__(pose)
        new_distance = int(self.rectangle/self.resolution) 
        valid = True
                
        edge_x = coord[0]
        edge_y = coord[1]
        for i in range (edge_x-new_distance, edge_x + new_distance  ):
            for j in range (edge_y - new_distance, edge_y + new_distance ):         
                if i < 0 or j < 0 or i >= (self.map.shape[0]) or j >= (self.map.shape[1]) :
                    return False
                else:
                    if self.map[i,j] >= -1:
                        valid = True
                    if self.map[i,j] >= 50:
                        return False
                              
        return valid
    
    
    

    def __position_to_map__(self, p):
        """This function converts a point from position coordinates to map coordinates

        Args:
            p (tuple): position of a point 

        Returns:
            tuple: coordinates of the point
        """

        new_x = int((p[0]-self.origin[0])/ self.resolution)
        new_y = int((p[1]-self.origin[1])/ self.resolution)
        coord = [new_x,new_y]
        if new_x < 0 or new_y < 0 or new_x >= self.map.shape[0] or new_y >= self.map.shape[1] :
        # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
            return []
        else:
            return coord
           
    def map_to_position(self, coord):
        """This function converts a point from map coordinates to position coordinates

        Args:
            coord (tuple): coordinates of a point 

        Returns:
            tuple: position of the point
        """
   
        new_x = (coord[0]*self.resolution) + self.origin[0]
        new_y = (coord[1]*self.resolution) + self.origin[1]
        p = [new_x,new_y]
        
        return p
    
    def Cluster_publisher(self,segment_points, marker_id, color):
        """This function publishes the frontiere points clustered to visualize them in the map

        Args:
            frontier_points (list): list of frontieres
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "world"  # Set the frame ID according to your map
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = marker_id
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0  # Fully opaque
        marker.scale.x = 0.1  # Line thickness
        # Initialize the orientation quaternion to identity
        marker.pose.orientation = Quaternion()
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        for point in segment_points:
            j = self.map_to_position(point)
            marker.points.append(Point(j[0], j[1],0))  # Add each point to the marker

        # Close the shape by connecting the last point to the first point
        fp = self.map_to_position(segment_points[0])
        first_point = Point(fp[0], fp[1],0)
        marker.points.append(first_point)

        return marker

       
    
    def frontier_publisher(self,frontier_points):
        """This function publishes the frontiere points to visualize them in the map

        Args:
            frontier_points (list): list of frontieres
        """

        frontiers = PoseArray()
        frontiers.header.frame_id = "world"
        frontiers.header.stamp = rospy.Time.now()
        j = []
    
        for i in frontier_points:
            j = self.map_to_position(i)
            point = Pose()
            point.position.x = j[0]
            point.position.y = j[1]
            
            frontiers.poses.append(point)
        
    
        self.frontier_pub.publish(frontiers)


    def goal_pubisher(self, point_to_go):
        """This Function publishes the goal position

        Args:
            point_to_go (Tuple): Position that the robot should go next to explore
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "world" # set the frame ID
        goal_pose.pose.position.x = point_to_go[0]  # set the position
        goal_pose.pose.position.y = point_to_go[1]
        
    
        self.goal_pub.publish(goal_pose)


    
    
    def bfs_frontier(self, map_array):
        """This is a Breadth first seatch function used to get the frontiers in a given map

        Args:
            map_array (array): map

        Returns:
            list: list of the frontiers
        """
        frontier_points = []
        visited = set()
        queue = deque()

        # Add free cells with unknown neighbors to the queue
        for row in range(self.map_size[0]):
            for col in range(self.map_size[1]):
                if map_array[row, col] == 0:
                    neighbors = [(row-1, col), (row+1, col), (row, col-1), (row, col+1), (row+1, col+1), (row-1, col+1), (row+1, col-1), (row-1, col-1) ]
                    is_frontier = False
                    for neighbor in neighbors:
                        n_row, n_col = neighbor
                        
                        if 0 <= n_row < self.map_size[0] and 0 <= n_col < self.map_size[1] and map_array[n_row, n_col] == -1:
                            is_frontier = True
                            queue.append(neighbor)
                            visited.add(neighbor)
                    if is_frontier == True:
                        frontier_points.append((row, col))

        return frontier_points

    def bfs_frontier_segment(self, frontier_points):
        """This function is clustring the frontiers

        Args:
            frontier_points (list): list of frontier points

        Returns:
            list of list of points: clustered points
        """
        segments = []
        visited = set()

        for point in frontier_points:
            if point not in visited:
                segment = []
                queue = deque([point])
                visited.add(point)

                while queue:
                    row, col = queue.popleft()
                    segment.append((row, col))
                    neighbors = [(row-1, col), (row+1, col), (row, col-1), (row, col+1), (row+1, col+1), (row-1, col+1), (row+1, col-1), (row-1, col-1) ]

                    for neighbor in neighbors:
                        n_row, n_col = neighbor
                        if (n_row, n_col) in frontier_points and (n_row, n_col) not in visited:
                            segment.append(neighbor)
                            queue.append(neighbor)
                            visited.add(neighbor)
                
                # Append the segment to the list of segments
                segments.append(list(segment))


        return segments




   

if __name__ == '__main__':
    rospy.init_node('explorer')  
    node = exploration()
   
    # Run forever
    rospy.spin()