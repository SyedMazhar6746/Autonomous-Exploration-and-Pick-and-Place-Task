#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
 
import numpy as np 
import math 
import rospy   
import tf
  
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from nav_msgs.msg import Odometry  
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
   

class Camera_Image:   
    def __init__(self):     
        self.image_pub = rospy.Publisher("/small_aruco_position", Float64MultiArray, queue_size=10)     
        self.i = 1     
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.dead_reckoning)              
        self.image_sub = rospy.Subscriber("/get_aruco_pose", Point, self.useless)                   
        self.image_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color", Image, self.imageCallback) 
                      
        # camaera position with respect to the robot   
        self.r_cx = 0.122      
        self.r_cy = -0.033   
        self.r_cz = 0.082    
        self.r_c_roll = math.pi/2 
        self.r_c_pitch = 0.0 
        self.r_c_yaw = math.pi/2 
        self.detected = 0
 
        self.marker_detected = False 

        self.camera = np.array([self.r_cx, self.r_cy, self.r_cz, self.r_c_roll, self.r_c_pitch, self.r_c_yaw])  
    
    def wrap_angle(self, angle):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


    def useless(self, useless): 
        # for simulation use the below line 
        self.image_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color", Image, self.imageCallback) 
        # for reality use the below line  
        # self.image_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/color/image_raw", Image, self.imageCallback)  
         
    def transform_r_c(self, x,y,z, roll, pitch, yaw):  
        Transf = np.eye((4)) 
        # Convert Euler angles to rotation matrix
        Rx = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], 
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])
        R = Rz @ Rx 

        # Create transformation matrix
        Trans = np.zeros((3,1))  
        # self.Trans[0:3,0:3] = R
        Trans[:,0] = x, y, z  
        # transformation matrix
        Transf[0:3, 0:3] = R
        Transf[0:3,3] = x, y, z   
        Trans = Trans.reshape(3,1)
        return Trans, R, Transf 
   
    def dead_reckoning(self, odom):   

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.robot_state = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, self.wrap_angle(yaw)]) 
 
        

    def imageCallback(self, Image_msg):  
        self.bridge = CvBridge()   
        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image_msg, "bgr8")  
        except CvBridgeError as e:
            print(e) 
 
        print('entered')   
        # Set marker ID and length   
        marker_id = 1 
        marker_length = 0.05          
  
        # Load the ArUco dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)  # for simulated box  
        # dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # for real box   

        camera_matrix = np.array([[1396.8086675255468, 0.0, 960.0],
                                 [0.0, 1396.8086675255468, 540.0],   
                                 [0.0, 0.0, 1.0]])      

        dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])       

        # Create a video capture object for the camera  
        frame = cv_image 

        # Detect ArUco markers in the frame
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, dictionary)

        # If at least one marker is detected, estimate its pose and draw an axis on it
        if marker_ids is not None and len(marker_ids) > 0: 
            for i in range(len(marker_ids)): 
                
                if marker_ids[i][0] == 1:      
                    # print('id', i)  
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs)       
                    rvecs = rvecs[0, :].reshape(1,3)    
                    tvecs = tvecs[0,:].reshape(1,3)     
                    cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids, (0, 255, 0)) 
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.05)   

                    # converted it to transformation matrix
                    Rot, _ = cv2.Rodrigues(rvecs)   
                    Transf = np.eye(4)
                    Trans = np.zeros((3,1))
                    Trans = tvecs[0,0], tvecs[0,1], tvecs[0,2]
                    Transf[0:3, 0:3] = Rot 
                    Transf[0:3,3] = np.squeeze(Trans)  

                    # transforming from camera to robot frame  
                    Trans_r_c, rot_r_c, Transf_r_c =  self.transform_r_c(self.camera[0], self.camera[1], self.camera[2], self.camera[3], self.camera[4], self.camera[5])     
                    # position of aruco wrt robot
                    Transf_r = Transf_r_c @ Transf  

                    # Display the X, Y, and Z coordinates of the marker on the frame
                    x = Transf_r[0,3]                          #+ 0.025  # 0.0375      
                    y = Transf_r[1,3] 
                    z = -0.22       
                    Trans_rob = np.array([[x],
                                          [y]]) 

                    Rot_z = np.array([[np.cos(self.robot_state[2]), -np.sin(self.robot_state[2])],
                                [np.sin(self.robot_state[2]), np.cos(self.robot_state[2])]])  
                    
                    Position_world = Rot_z @ Trans_rob

                    x = self.robot_state[0] + Position_world[0,0]
                    y = self.robot_state[1] + Position_world[1,0]  

                    # Check the data types and convert to float if necessary
                    if isinstance(x, tuple):     
                        x = float(x[0]) 
                    if isinstance(y, tuple):
                        y = float(y[0])
                    if isinstance(z, tuple):  
                        z = float(z[0])  

                    # Create a Float64MultiArray message to publish the point values 
                    self.detected = self.detected +1 
                    if self.detected == 13:   
                        point_msg = Float64MultiArray()

                        # Set the x, y, and z values of the point
                        print('aruco position', float(x), float(y), float(z), 0.0)  
                        point_msg.data = [float(x), float(y), float(z), 0.0]       # float(marker_ids[i][0])  

                        # Publish the point message
                        self.image_pub.publish(point_msg)   
                        rospy.signal_shutdown("ArUco marker detected in exploration")      
        
          

if __name__ == '__main__':  
   
    try:
        rospy.init_node("image_sub_d", anonymous=True)    
        Camera_Image()  
        
        rospy.spin()   
        
    except rospy.ROSInterruptException:
        pass
