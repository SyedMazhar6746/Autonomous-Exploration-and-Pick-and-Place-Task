#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np 
import math  
import rospy  

from sensor_msgs.msg import NavSatFix, Image 
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from stonefish_ros.msg import BeaconInfo 
   
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped  
 
class Camera_Image: 
    def __init__(self):   
        self.image_pub = rospy.Publisher("/aruco_position", Float64MultiArray, queue_size=10)     
        self.i = 1   
        self.image_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color", Image, self.imageCallback)     
        
    def imageCallback(self, Image_msg): 
        # print(Image_msg)  
        self.bridge = CvBridge()   
        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image_msg, "bgr8")  
        except CvBridgeError as e:
            print(e) 
        

        # Set marker ID and length 
        marker_id = 1
        marker_length = 0.16 

        # Load the ArUco dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        
        camera_matrix = np.array([[1396.8086675255468, 0.0, 960.0],
                                 [0.0, 1396.8086675255468, 540.0],   
                                 [0.0, 0.0, 1.0]]) 

        dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])   

        # Create a video capture object for the camera
        frame = cv_image 

        
 
        # Create a named window for displaying the video stream
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL) 
        cv2.resizeWindow("Camera", 800, 600)          

        

        # Detect ArUco markers in the frame
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, dictionary)
        
        # If at least one marker is detected, estimate its pose and draw an axis on it
        if marker_ids is not None and len(marker_ids) > 0:
            for i in range(len(marker_ids)):
                # print(marker_ids[i]) 
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs)   
                    
                rvecs = rvecs[0, :].reshape(1,3)    
                tvecs = tvecs[0,:].reshape(1,3)   
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids, (0, 255, 0)) 
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.05)   

                # Display the X, Y, and Z coordinates of the marker on the frame

                x = tvecs[0][0]
                y = tvecs[0][1] 
                z = tvecs[0][2]      
                cv2.putText(frame, f"X: {x}", (10, 35), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Y: {y}", (10, 60), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Z: {z}", (10, 85), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)   

                # Create a Float64MultiArray message to publish the point values 
                point_msg = Float64MultiArray()

                # Set the x, y, and z values of the point
                point_msg.data = [x, y, z, float(marker_ids[i])]    

                # Publish the point message
                self.image_pub.publish(point_msg)  


        # Display the frame on the screen
        cv2.imshow("Camera", frame)  
        cv2.waitKey(3)   

if __name__ == '__main__': 
 

    try:
        rospy.init_node("image_sub1")   
        Camera_Image()   
        rospy.spin()   
    except rospy.ROSInterruptException:
        pass
