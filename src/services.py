#!/usr/bin/python3
 
import rospy
import numpy as np
import rospy 
from std_msgs.msg import Float64MultiArray  
from geometry_msgs.msg import Point
from std_msgs.msg import Int32 
from  mob_hands_on_func_call_2 import *  
from h_invention.srv import intervention, interventionResponse 
import rospy 


class ManageObject():
    def __init__(self): 


        self.weight = None
        self.goal = None   
            
        self.pub_weight = rospy.Publisher('/weight_set', Float64MultiArray, 
                                          queue_size=10)
        self.pub_goal = rospy.Publisher('/goal_set', Float64MultiArray, queue_size=10) 
        self.pub_aruco = rospy.Publisher('/get_aruco_pose', Point, queue_size=10)   

        self.pub_task = rospy.Publisher('/task_set', Int32,  queue_size=10)   

        server_weight = rospy.Service('weight_server', intervention,  
                                     self.weight_set) 
        
        server_aruco = rospy.Service('aruco_server', intervention,  
                                     self.aruco_set)
        
        server_goal = rospy.Service('goal_server', intervention,  
                                     self.goal_set)
        
        server_task = rospy.Service('task_server', intervention,  
                                self.task_set)  


    def weight_set(self, srv_msg): 
        weight_gain = srv_msg.data 
        print('weight', weight_gain) 
        # setting the parameter
        rospy.set_param('weighted_DLS', weight_gain)   
        # publishing the set value 
        weighted_DLS = Float64MultiArray()
        weighted_DLS.data = weight_gain    # take the value from client        
        self.pub_weight.publish(weighted_DLS)  
        return interventionResponse()  
    
    def task_set(self, task_msg):  #  integer value 0 or 1 or 2  
        print(task_msg.data[0]) 
        selected_index = int(task_msg.data[0]) 
        print('selected_index', selected_index)  
        # # publishing the set value  
        task_va = Int32() 
        task_va.data = selected_index    # take the value from client        
        self.pub_task.publish(task_va)  
        return interventionResponse()  
    
    def goal_set(self, srv_msg):  
        print('goal in service', srv_msg) 
        goal = srv_msg.data 
        print('goal', goal)  
        # setting the parameter 
        rospy.set_param('goal', goal)    
        # publishing the set value  
        goal_value = Float64MultiArray()
        goal_value.data = goal    # take the value from client          
        self.pub_goal.publish(goal_value)   
        return interventionResponse() 
        
    
    def aruco_set(self, srv_msg):
        point_msg = Point()
        point_msg.x = 1.0
        self.pub_aruco.publish(point_msg)   
        return interventionResponse()
    
if __name__ == '__main__':

    rospy.init_node('intervention_service') 
    check_object = ManageObject()
    rospy.spin()  
    