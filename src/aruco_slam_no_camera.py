#!/usr/bin/python

# working with one feature everything hardcored  
 
import numpy as np 
import math 
import math as m 
import rospy    
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState, Imu 
from sensor_msgs.msg import NavSatFix 
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray  
from stonefish_ros.msg import BeaconInfo 
import transforms3d.euler as euler
   
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  
    
class DifferentialDrive: 
    def __init__(self):     

        # camera position with respect to the robot 
        self.r_cx = 0.122    
        self.r_cy = -0.033      
        self.r_cz = 0.082
        self.r_c_roll = math.pi/2 
        self.r_c_pitch = 0.0 
        self.r_c_yaw = math.pi/2 
        
        self.v = 0 
        self.w = 0 

        self.m = 1 
        self.Zk = 0 
         
        self.tf_br = TransformBroadcaster()    
        # robot constants 
        self.wheel_radius = 0.035 # meters       
        self.wheel_base_distance = 0.23 # meters  

        self.camera = np.array([self.r_cx, self.r_cy, self.r_cz, self.r_c_roll, self.r_c_pitch, self.r_c_yaw]) 
 
        # initial pose of the robot  
        self.Xk = np.zeros([4,1])    
        
        # velocity and angular velocity of the robot 
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.current_time = 0.0 

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False

        self.last_time = rospy.Time.now()

        self.id = 0 
        self.beacon_list = []  
        self.rz = 0.0 # robot state z component  

        self.Hk = np.zeros((3,4))   
        self.F1 = np.zeros((4,4))
        self.F1_I = np.diag((1,1,1))   
        self.F2 =  np.zeros((4,2)) 
        self.F2_o = np.zeros((3,3))   
        self.g1 = np.zeros((4,1)) 
        self.g2 = np.zeros((4,3)) 
        
        self.num = 0
        self.num_2 = 0

        self.map12 = {}  
        self.rate = rospy.Rate(10)    

        # covariance details

        # uncertatinty in the position  
        self.Pk = np.array([[0.1**2, 0, 0, 0],    # 2,2 matrix  
                            [0, 0.1**2, 0, 0],  
                            [0, 0, 0.0, 0],  
                            [0, 0, 0, 0.05**2]])       # 3,3 matrix with diagonal 0.04           
        
        # uncertatinty in the wheels
        self.Qk = np.array([[0.01**2, 0],    # 2,2 matrix     
                             [0, 0.01**2]])      

        # uncertatinty in the observation 
        self.Rk =  np.array([[0.2**2, 0, 0],    # 2,2 matrix        
                             [0, 0.4**2, 0],  
                             [0, 0, 0.4**2]])  

        # Ppublisher  
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)     
        self.marker_pub = rospy.Publisher("/beacons_viz", MarkerArray, queue_size=10) 
        self.marker_pub1 = rospy.Publisher('/gps/ellipsoid_marker', Marker, queue_size=10) 
        
         
        # Subscribers  
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback)  
        self.js_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/imu", Imu, self.Imu_callback)   
        self.aruco_pose = rospy.Subscriber("/aruco_position", Float64MultiArray, self.aruco_position)             
 

    def Imu_callback(self, Imu_msg):  
        self.imu_vel = Imu_msg.angular_velocity.z
        self.imu_quaternion = (Imu_msg.orientation.x, Imu_msg.orientation.y, Imu_msg.orientation.z, Imu_msg.orientation.w) 
        self.Imu_angle = euler_from_quaternion(self.imu_quaternion)  
        self.Imu_theta = self.Imu_angle[2]

        # observation from IMU angle  
        self.Zk = self.Imu_theta #+ math.pi/2 # for circuit1    
        Vk = 1     
        Rk = 0.02**2 

        I = np.eye(self.Pk.shape[0])        
        Hk = np.zeros((1,self.Pk.shape[0]))    
        Hk[0,3] = 1 

        h_x = np.dot(Hk,self.Xk) 
        h_x = self.wrap_angle(h_x)  
        Ykn_angle = self.Zk - np.dot(Hk,self.Xk) 
       
         
        before_inv = (Hk @ (self.Pk @ Hk.T)) + (Vk * Rk * Vk)  
        inv = np.linalg.inv(before_inv)  
        K_gain = ((self.Pk @ Hk.T) @ inv)     
        

        self.Xk = self.Xk + K_gain @ (Ykn_angle)    
          
        self.Pk = ((I - (K_gain @ Hk))@ self.Pk)    

           
    def transform_r_c(self, x,y,z, roll, pitch, yaw):
        Transf = np.eye((4)) 
        # Convert Euler angles to rotation matrix
        Rx = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],  
                    [math.sin(yaw), math.cos(yaw), 0], 
                    [0, 0, 1]])
        R = Rz @ Rx  

        # Create transformation matrix 
        Trans = np.zeros((3,1))  
        # self.Trans[0:3,0:3] = R 
        Trans[0,0] = x  
        Trans[1,0] = y
        Trans[2,0] = z
        # transformation matrix 
        Transf[0:3, 0:3] = R
        Transf[0,3] = x 
        Transf[1,3] = y 
        Transf[2,3] = z  
        return Trans, R, Transf 
    
    def transform_r_c_inv(self, x,y,z, roll, pitch, yaw):
        Transf = np.eye((4)) 
        # Convert Euler angles to rotation matrix
        Rx = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], 
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])
        R = Rz @ Rx 

        # Create transformation matrix 
        Trans = np.zeros((3,1))  
        Trans[0,0] = x  
        Trans[1,0] = y
        Trans[2,0] = z
        Trans = Trans.reshape(3,1) 

        R_T = R.T
        Trans_T = -R_T @ Trans  
        Trans_T = Trans_T.reshape(3,1) 
        # transformation matrix
        Transf[0:3, 0:3] = R_T

        Transf[0,3] = Trans_T[0] 
        Transf[1,3] = Trans_T[1] 
        Transf[2,3] = Trans_T[2]  
 
        return Trans_T, R_T, Transf   

    def wrap_angle(self, ang):
        if isinstance(ang, np.ndarray):
            ang[ang > np.pi] -= 2 * np.pi
            return ang
        else: 
            return ang + (2.0 * math.pi * math.floor((math.pi - ang) / (2.0 * math.pi)))

    def aruco_position(self, beacon): 
        # # feature in the robot frame 

        self.c_fx = beacon.data[0] # this is the observed value  (beacon.x) 
        self.c_fy = beacon.data[1]
        self.c_fz = beacon.data[2]     
        self.id = beacon.data[3]  
        if  self.id != 0 and self.id != 1023.0 and self.id != 1.0 and self.id != 31.0 and self.id != 71.0:     # self.c_fy < 0 and         for  remmoving aruco beneath the ground   
            
            self.c_f = np.array([[self.c_fx], 
                            [self.c_fy],     
                            [self.c_fz]])    
     
            #--------------------------------------------------------------------------------------------------------------------------------
              
             
            self.range =  math.sqrt(self.c_fx**2 + self.c_fy**2 + self.c_fz**2) 
            self.azimuth = math.atan2(self.c_fy, self.c_fx) 
            self.azimuth = self.wrap_angle(self.azimuth)  
            self.elevation = math.atan2(self.c_fz, math.sqrt(self.c_fx**2 + self.c_fy**2))       
            self.elevation = self.wrap_angle(self.elevation)   

            J_p2c = np.array([[np.cos(self.elevation)*np.cos(self.azimuth), -self.range*np.cos(self.elevation)*np.sin(self.azimuth), -self.range*np.sin(self.elevation)*np.cos(self.azimuth)],
                            [np.cos(self.elevation)*np.sin(self.azimuth), self.range*np.cos(self.elevation)*np.cos(self.azimuth), -self.range*np.sin(self.elevation)*np.sin(self.azimuth)], 
                            [np.sin(self.azimuth), 0 , self.range*np.cos(self.elevation)]])  

            Rk_c = J_p2c @ self.Rk @ J_p2c.T   # uncertainty in camera frame in cartesian coordinates   

            self.Xk, self.Pk  = self.update(self.c_f, Rk_c, self.id)    # take in Xk and Pk        
            self.quad(self.Xk, self.Pk, self.current_time, self.v, self.w)       
        
    def update(self, c_f, Rk_c, id):   
        self.c_f = c_f.reshape(3,1) 
        self.c_fx = c_f[0,0]  
        self.c_fy = c_f[1,0]  
        self.c_fz = c_f[2,0]   
        Rk_c = Rk_c 
        self.id = id
        roll = self.camera[3]  
        yaw = self.camera[5]  
        global fx_x, fx_y, fxx_x, fxx_y, Zk, Rk_r  
        
        if self.id not in self.beacon_list: 
            if self.id not in self.beacon_list:   
                
                # wrapping the angle   
                self.Xk[3,0] = self.wrap_angle(self.Xk[3,0]) 

                self.beacon_list.append(self.id)  
                print('self.beacon_list', self.beacon_list)   
                # increasing the matrix size
                self.Xk = np.pad(self.Xk, ((0, 3), (0, 0)))  # 5 by 1  
                self.Hk = np.pad(self.Hk, ((0, 0), (0, 3)))  
                self.g1 = np.pad(self.g1, ((0, 3), (0, 3)))  
                self.g2 = np.pad(self.g2, ((0, 3), (0, 0)))   


                self.F1 = np.pad(self.F1, ((0, 3), (0, 3)))
                self.F1[-3:,-3:] = self.F1_I  
                self.F2 = np.pad(self.F2, ((0, 3), (0, 0)))  
                
                I4 = np.eye(4)
                I3 = np.eye(3)  
                O = np.zeros((3,2))
                Vk = np.eye(3)   

                # --
                Trans, rot, _ =  self.transform_r_c(self.camera[0], self.camera[1], self.camera[2], self.camera[3], self.camera[4], self.camera[5])    

                self.rm_r_c = Trans + rot @ self.c_f 

                # position of the feature in robot frame 
                self.x_b = self.rm_r_c[0,0]
                self.y_b = self.rm_r_c[1,0]  
                self.z_b = self.rm_r_c[2,0]   

                # partial derivative of camera to robot compounding with respect to feature_in_camera  
                J2 = rot    

                Rk_r = J2 @ Rk_c @ J2.T   # uncertainty in robot frame in cartesian coordinates   

                
                # position of the feature in robot frame in cartesian coordinate  
                Zk = np.array([[self.x_b],[self.y_b], [self.z_b]]) 
                # --

                th = self.Xk[3,0]
                # feature in the world frame 
                Zk_p = np.array([[self.Xk[0,0] + Zk[0,0]*np.cos(th) - Zk[1,0]*np.sin(th)],   # add in Xk
                                [self.Xk[1,0] + Zk[0,0]*np.sin(th) + Zk[1,0]*np.cos(th)], 
                                [self.Xk[2,0] + Zk[2,0]]])   

                self.Xk[-3:] = Zk_p  # fill only last three rows    

                #   -----------------------------------------------------------------



                fx_x = self.c_fy*np.sin(yaw)*np.sin(roll)+self.c_fz*np.sin(yaw)*np.cos(roll)  # x wrt roll
                fx_y = -self.c_fy*np.cos(yaw)*np.sin(roll)-self.c_fz*np.cos(yaw)*np.cos(roll)  # y wrt roll

                J_1_p = np.array([[1, 0, 0, -Zk[0,0]*np.sin(th)-Zk[1,0]*np.cos(th)],    
                                  [0, 1, 0,  Zk[0,0]*np.cos(th)-Zk[1,0]*np.sin(th)],    
                                  [0, 0, 1, 0]])    
                
                J_2_p = np.array([[np.cos(th), -np.sin(th), 0], 
                                [np.sin(th), np.cos(th), 0], 
                                [0, 0, 1]]) 
            

                # accumulated uncertainty  
                if self.m == 1: 
                    self.g1[0:4, 0:4] =  I4 
                    self.g1[-3:,:4] = J_1_p    
                    self.m = 2    
                else :
                    self.g1[-6:-3,-3:] = I3 
                    self.g1[-3:,:4] = J_1_p    

                self.g2[:,:] = 0       # 5 by 2  
                self.g2[-3:,:] = J_2_p  # 5 by 2    

                self.Pk = self.g1 @ self.Pk @ self.g1.T + self.g2 @ Rk_r @ self.g2.T  
                self.g1[-3:,:4] = 0     

                # increasing the map size to include features      
                self.map12[self.num_2] = 0.0   

                self.num_2 += 1   
  
        else:   
             
            idx = self.beacon_list.index(self.id) 
            f_i_1, f_i_2 = 4 + 3*idx, 7 + 3*idx  # index of the feature in the state vector (x,y,z) for feature 1, it is 4 to 7 
 
            self.Xk[3,0] = self.wrap_angle(self.Xk[3,0])  
            new_angle  = self.Xk[3,0] 
            
            # position of the feature in camera frame in cartesian coordinate   
            Zk1 = self.c_f  

            I_1 = np.eye(self.Pk.shape[0]) 
            Vk = np.eye(3) 

            map = self.Xk[f_i_1: f_i_2,:] ###  # 3 by 1 map of feature 1 (matrix)     

            th = self.Xk[3,0] 
            
            # ---------------------------  
            roll = math.pi/2
            yaw = math.pi/2  
            th = self.Xk[3,0]
            # robot in world frame 
            n_X_r = self.Xk[0,0]
            n_Y_r = self.Xk[1,0]
            n_Z_r = self.Xk[2,0]
            # camera wrt robot 
            r_X_c = self.camera[0]
            r_Y_c = self.camera[1]
            r_Z_c = self.camera[2] 

            c_X_r = -r_X_c*np.cos(yaw) - r_Y_c*np.sin(yaw)
            c_Y_r = r_X_c*np.sin(yaw)*np.cos(roll) - r_Y_c*np.cos(yaw)*np.cos(roll) - r_Z_c*np.sin(roll) 
            c_Z_r = -r_X_c*np.sin(yaw)*np.sin(roll)  + r_Y_c*np.sin(roll)*np.cos(yaw) - r_Z_c*np.cos(roll)   
            sigma_1 = map[1,0]*np.cos(th)- n_Y_r*np.cos(th)-map[0,0]*np.sin(th)+ n_X_r*np.sin(th) 
            sigma_2 = map[0,0]*np.cos(th)- n_X_r*np.cos(th)+map[1,0]*np.sin(th)- n_Y_r*np.sin(th)   
 

            h_x = np.array([[c_X_r + sigma_1*np.sin(yaw) + sigma_2*np.cos(yaw) ], 
                           [c_Y_r + np.sin(roll)*(map[2,0]-n_Z_r) + np.cos(roll)*np.cos(yaw)*sigma_1 - np.cos(roll)*np.sin(yaw)*sigma_2],
                           [c_Z_r + np.cos(roll)*(map[2,0]-n_Z_r) - np.cos(yaw)*np.sin(roll)*sigma_1 + np.sin(roll)*np.sin(yaw)*sigma_2]])  
            # -------------------------------------------
            # innovation 
            Ykn = (Zk1 - h_x)   

            # ------------------------------------------------------------------------- 
            self.Hk[:,:] = 0 
            roll = self.camera[3]  
            yaw = self.camera[5]      
            th = self.Xk[3,0] 

            repeat = np.array([[np.cos(yaw)*np.cos(th)-np.sin(yaw)*np.sin(th),                              np.cos(yaw)*np.sin(th)+np.sin(yaw)*np.cos(th),                           0], 
                               [-np.cos(roll)*np.cos(yaw)*np.sin(th)-np.cos(roll)*np.sin(yaw)*np.cos(th),   np.cos(roll)*np.cos(yaw)*np.cos(th)-np.cos(roll)*np.sin(yaw)*np.sin(th), np.sin(roll)], 
                               [np.sin(roll)*np.cos(yaw)*np.sin(th)+np.sin(roll)*np.sin(yaw)*np.cos(th),    np.sin(roll)*np.sin(yaw)*np.sin(th)-np.sin(roll)*np.cos(yaw)*np.cos(th), np.cos(roll)]])     
 
 
            f1_th = -map[1,0]*np.sin(th) + self.Xk[1,0]*np.sin(th) - map[0,0]*np.cos(th) + self.Xk[0,0]*np.cos(th)  
            f2_th = -map[0,0]*np.sin(th) + self.Xk[0,0]*np.sin(th) + map[1,0]*np.cos(th) - self.Xk[1,0]*np.cos(th) 

            self.Hk[:,0:4] = np.array([[-np.cos(th)*np.cos(yaw)+np.sin(yaw)*np.sin(th),                                 -np.cos(yaw)*np.sin(th)-np.sin(yaw)*np.cos(th),                                 0,                 np.cos(yaw)*f2_th+np.sin(yaw)*f1_th],                                       

                                        [np.cos(roll)*np.cos(yaw)*np.sin(th)+np.cos(roll)*np.sin(yaw)*np.cos(th),        -np.cos(roll)*np.cos(yaw)*np.cos(th)+np.cos(roll)*np.sin(yaw)*np.sin(th),       -np.sin(roll),      np.cos(roll)*np.cos(yaw)*f1_th-np.cos(roll)*np.sin(yaw)*f2_th], 
                                        
                                        [-np.sin(roll)*np.cos(yaw)*np.sin(th)-np.sin(roll)*np.sin(yaw)*np.cos(th),        np.sin(roll)*np.cos(yaw)*np.cos(th)-np.sin(roll)*np.sin(yaw)*np.sin(th),       -np.cos(roll),     -np.sin(roll)*np.cos(yaw)*f1_th+np.sin(roll)*np.sin(yaw)*f2_th]])        
 
            self.Hk[:,f_i_1:f_i_2] = repeat         
    

        
            before_inv = (self.Hk @ (self.Pk @ self.Hk.T)) + (Vk @ (Rk_c @ Vk.T))       
            inv = np.linalg.pinv(before_inv)    
            K_gain = ((self.Pk @ self.Hk.T) @ inv) 
             
            self.Xk = self.Xk + K_gain @ (Ykn)  

            self.Xk[3,0] = self.Zk           
            self.Pk = ((I_1 - (K_gain @ self.Hk))@ self.Pk @ (I_1 - (K_gain @ self.Hk)).T)    
            
            # visualizing the aruco as a point in the simulation 
            for m in range(len(self.map12)):    
                 self.map12[m] = [self.Xk[3*m+4,0], self.Xk[3*m+5,0], self.Xk[3*m+6,0]]     
            self.modem_visualization(self.map12)     
             
       
        return self.Xk, self.Pk 
    

    def modem_visualization(self, map): # feature contain all the modem positions as a column vector   
         
        self.map = map
        

        ma = MarkerArray()
        

        for i in range(len(self.map)):   

            idx = list(self.map.keys())[i]
             
            marker = Marker()
            marker.header.frame_id = "world"   
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.id = i*2 

            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = self.map[idx][0]
            marker.pose.position.y = self.map[idx][1]
            marker.pose.position.z = self.map[idx][2] 
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1 
            marker.scale.y = 0.1
            marker.scale.z = 0.1  

            marker.color.r = 1.0
            marker.color.a = 0.9

            ma.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "world"  
            marker_text.type = marker_text.TEXT_VIEW_FACING
            marker_text.action = marker_text.ADD
            marker_text.id = i*2 + 1 

            marker_text.header.stamp = rospy.Time.now()
            marker_text.pose.position.x = self.map[idx][0]
            marker_text.pose.position.y = self.map[idx][1]
            marker_text.pose.position.z = self.map[idx][2]  + 1.0  
            marker_text.pose.orientation.w = 1.0

            marker_text.scale.x = 0.1
            marker_text.scale.y = 0.1
            marker_text.scale.z = 0.3
        

            marker_text.color.r = 1.0
            marker_text.color.a = 0.9

            marker_text.text = "Aruco "+str(idx) 

            ma.markers.append(marker_text)

            # uncertainty of
            self.cov = self.Pk[i*3+4:i*3+7,i*3+4:i*3+7 ]  

            # Assuming R is the rotation matrix
            eigenvalues, eigenvectors = np.linalg.eig(self.cov)

            # Find the eigenvector associated with the largest eigenvalue
            major_axis_eigenvector = eigenvectors[:, np.argmax(eigenvalues)]

            # Compute the roll, pitch, and yaw angles
            roll_m = np.arctan2(major_axis_eigenvector[1], major_axis_eigenvector[0])

            # Calculate major and minor axis lengths
            confidence_factor = 2.0  # Change this value for different confidence levels
            major_axis_length = confidence_factor * np.sqrt(eigenvalues[np.argmax(eigenvalues)])
            minor_axis_length = confidence_factor * np.sqrt(eigenvalues[np.argmin(eigenvalues)])

            # Calculate the orientation angle
            orientation_angle = np.arctan2(major_axis_eigenvector[1], major_axis_eigenvector[0])

            pitch_m = np.arcsin(-major_axis_eigenvector[2])
            yaw_m = np.arctan2(self.cov[2, 1], self.cov[2, 2]) 

            quat = quaternion_from_euler(roll_m, pitch_m, yaw_m)

            marker1 = Marker() 
            marker1.header.frame_id = 'world'   
            marker1.header.stamp = rospy.Time.now()
            marker1.ns = 'feature_ellipsoid' 
            marker1.id = i*100+1
            marker1.type = Marker.SPHERE 
            marker1.action = Marker.ADD
            marker1.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) 

            # Set marker position to GPS fix location 
            marker1.pose.position.x = self.map[idx][0]
            marker1.pose.position.y = self.map[idx][1]  
            marker1.pose.position.z = self.map[idx][2]  

            marker1.pose.orientation.x = 0.0
            marker1.pose.orientation.y = 0.0  
            marker1.pose.orientation.z = 0.0
            marker1.pose.orientation.w = 1.0  

            # Set marker scale to represent measurement uncertainty
            #   
            marker1.scale.x = major_axis_length
            marker1.scale.y = minor_axis_length 
            marker1.scale.z = 0.1   
 
            ma.markers.append(marker1)    
        self.marker_pub.publish(ma)    
               

    def prediction(self, left_wheel_velocity, right_wheel_velocity, msg_sec, msg_nsec): 
        
        # 
        # print('predicting') 
        self.left_wheel_velocity = left_wheel_velocity   
        self.right_wheel_velocity = right_wheel_velocity

        left_lin_vel = self.left_wheel_velocity * self.wheel_radius
        right_lin_vel = self.right_wheel_velocity * self.wheel_radius

        self.v = (left_lin_vel + right_lin_vel) / 2.0      
        self.w = (left_lin_vel - right_lin_vel) / self.wheel_base_distance  
          
        #calculate dt  
        self.current_time = rospy.Time.from_sec(msg_sec + msg_nsec * 1e-9)  
        dt = (self.current_time - self.last_time).to_sec()  
        self.last_time = self.current_time     


        self.Xk[3,0] = self.wrap_angle(self.Xk[3,0])  

        self.Xk[0,0] = self.Xk[0,0] + np.cos(self.Xk[3,0]) * self.v * dt 
        self.Xk[1,0] = self.Xk[1,0] + np.sin(self.Xk[3,0]) * self.v * dt  
        self.Xk[2,0] = self.rz 
        self.Xk[3,0] = self.Zk      
        self.Xk[3,0] = self.wrap_angle(self.Xk[3,0])  # wrapping the angle 

                        
        Ak = np.array([[1, 0, 0, -np.sin(self.Xk[3,0])*self.v*dt], 
                        [0, 1, 0, np.cos(self.Xk[3,0])*self.v*dt],   
                        [0, 0, 1, 0],   
                        [0, 0, 0, 1]])    
 
        Bk = np.array([[np.cos(self.Xk[3,0])*dt*0.5*self.wheel_radius,  np.cos(self.Xk[3,0])*dt*0.5*self.wheel_radius],    
                       [np.sin(self.Xk[3,0])*dt*0.5*self.wheel_radius,  np.sin(self.Xk[3,0])*dt*0.5*self.wheel_radius], 
                       [0, 0],  
                       [(dt*self.wheel_radius)/self.wheel_base_distance, -(dt*self.wheel_radius)/self.wheel_base_distance]])            
                   

        self.F1[0:4,0:4] = Ak
        self.F2[0:4,:] = Bk      
             
        self.Pk = self.F1 @ self.Pk @ self.F1.T + self.F2 @ self.Qk @ self.F2.T  

        return self.Xk, self.Pk       
 
    def quad(self, Xk, Pk, current_time, v, w): 
                self.Xk = Xk
                self.Pk = Pk 
                self.v = v 
                self.w = w
                self.current_time = current_time 
 
                q = quaternion_from_euler(0, 0, self.Xk[3,0])   
 
                odom = Odometry() 
                odom.header.stamp = self.current_time    
                odom.header.frame_id = "world"      
                odom.child_frame_id = "turtlebot/kobuki/base_footprint"    

                odom.pose.pose.position.x =  self.Xk[0,0]  
                odom.pose.pose.position.y =  self.Xk[1,0] 
                odom.pose.pose.position.z =  self.Xk[2,0] 

                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1] 
                odom.pose.pose.orientation.z = q[2] 
                odom.pose.pose.orientation.w = q[3]   

                odom.pose.covariance = [math.sqrt(abs(self.Pk[0,0])), math.sqrt(abs(self.Pk[0,1])), math.sqrt(abs(self.Pk[0,2])), 0.0, 0.0, math.sqrt(abs(self.Pk[0,3])),   
                                        math.sqrt(abs(self.Pk[1,0])), math.sqrt(abs(self.Pk[1,1])), math.sqrt(abs(self.Pk[1,2])), 0.0, 0.0, math.sqrt(abs(self.Pk[1,3])),   
                                        math.sqrt(abs(self.Pk[2,0])), math.sqrt(abs(self.Pk[2,1])), math.sqrt(abs(self.Pk[2,2])), 0.0, 0.0, math.sqrt(abs(self.Pk[2,3])),      
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
                                        math.sqrt(abs(self.Pk[3,0])), math.sqrt(abs(self.Pk[3,1])), math.sqrt(abs(self.Pk[3,2])), 0.0, 0.0, math.sqrt(abs(self.Pk[3,3]))]   

 
                odom.twist.twist.linear.x = self.v    
                odom.twist.twist.angular.z = self.w     
                 
                self.odom_pub.publish(odom)    
                self.tf_br.sendTransform((self.Xk[0,0], self.Xk[1,0], self.Xk[2,0]), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)    
                
    def joint_state_callback(self, msg):  
 
        if msg.name[0] == "turtlebot/kobuki/wheel_left_joint": 
            self.left_wheel_velocity = msg.velocity[0]
            self.left_wheel_received = True
            return 
        elif msg.name[0] == "turtlebot/kobuki/wheel_right_joint":  
            self.right_wheel_velocity = msg.velocity[0] 
            if (self.left_wheel_received): 
                # Reset flag 
                self.left_wheel_received = False        
                msg_sec = msg.header.stamp.secs 
                msg_nsec = msg.header.stamp.nsecs
 
                self.Xk, self.Pk =  self.prediction(self.left_wheel_velocity, self.right_wheel_velocity, msg_sec, msg_nsec)     

                self.quad(self.Xk, self.Pk, self.current_time, self.v, self.w)           
                 

   
if __name__ == '__main__':  
  
    rospy.init_node("differential_drive", anonymous=True)    
    robot = DifferentialDrive()       
    rospy.spin()  

    





