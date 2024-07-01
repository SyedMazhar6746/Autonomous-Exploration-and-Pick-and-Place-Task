#!/usr/bin/python3

import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import py_trees.decorators
import py_trees.display
import time
from nav_msgs.msg import Odometry  
from std_msgs.msg import Float64MultiArray 
import math 
from h_invention.srv import intervention 
import numpy as np  
from std_srvs.srv import SetBool
import signal
   
import tf 

class Move_to_points(py_trees.behaviour.Behaviour):

    # success if distance between goal and end-effector is less than threshold  
    def __init__(self, name):
        super(Move_to_points, self).__init__(name) 
        self.sub_pose_EE = rospy.Subscriber('pose_EE', PoseStamped, self.EE_pose)
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.dead_reckoning) 
        time.sleep(2)  
    def setup(self):
        self.on = False 
        self.ind = 0
        self.logger.debug("  %s [Move_to_points::setup()]" % self.name)
        rospy.wait_for_service('goal_server')  
        rospy.wait_for_service('weight_server')  
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump') 
        try:
            # Create a proxy for the service
            self.set_pump_proxy = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            self.set_goal = rospy.ServiceProxy('goal_server', intervention) 
            self.set_weight = rospy.ServiceProxy('weight_server', intervention) 
 
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e: 
            self.logger.debug("  %s [Move_to_points::setup()]" % self.name)  
 
    def initialise(self): 
        self.goal_xyz = 0 
        self.distance = 0  
        self.ind = self.ind + 1  
        self.weight = [1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]   
        self.angle = float(math.radians(0))  
        self.goal_position = blackboard.aruco_pos  


        distance_behind = 0.2  

        opposite_angle = self.robot_state[2] + math.pi 
    
        # Calculate the x and y components of the point behind the robot 
        x_behind = self.robot_state[0] + distance_behind * math.cos(opposite_angle)
        y_behind = self.robot_state[1] + distance_behind * math.sin(opposite_angle)

        distance_side = 0.22 

        side_angle = self.robot_state[2] - math.pi/2

        x_side = self.robot_state[0] + distance_side * math.cos(side_angle) 
        y_side = self.robot_state[1] + distance_side * math.sin(side_angle) 

        pick_1 = [self.goal_position[0], self.goal_position[1], -0.15, self.angle]    
        pick_2 = [self.goal_position[0]+0.05, self.goal_position[1], -0.3, self.angle]
        pick_3 = [x_side, y_side, -0.365, self.angle]  
        # for simulation 
        pick_4 = [x_behind, y_behind, -0.36, self.angle]   
        pick_5 = [x_behind, y_behind, -0.36, self.angle]    
        pick_6 = [x_behind, y_behind, -0.36, self.angle] 

        front_distance = 0.3
        front_theta = self.robot_state[2] - math.pi/6 
        front_point_x = self.robot_state[0] + front_distance * math.cos(front_theta)
        front_point_y = self.robot_state[1] + front_distance * math.sin(front_theta)   
        
        place_1 = [x_behind, y_behind, -0.36, self.angle] 
        place_2 = [x_behind, y_behind, -0.36, self.angle] 
        place_3 = [x_side, y_side, -0.36, self.angle]       

        place_4 = [front_point_x, front_point_y, -0.3, self.angle]      
        place_5 = [front_point_x, front_point_y, -0.144, self.angle]   
        place_6 = [front_point_x, front_point_y, -0.3, self.angle]              


        if self.ind == 1:  
            self.point_locations = [pick_1, pick_2, pick_3, pick_4, pick_5, pick_6]  
        if self.ind == 2:
            self.point_locations = [place_1, place_2, place_3, place_4, place_5, place_6] 
            self.on = True 

        self.logger.debug("  %s [Move_to_points::initialise()]" % self.name)        
  
    def update(self):
        try: 
            signal.signal(signal.SIGINT, self.signal_handler)
            response = self.set_weight(self.weight)  
            rospy.loginfo("Weight set successfully")   
            if self.point_locations != 0:
                for i in range(len(self.point_locations)):    
                    self.logger.debug("  {}: Publishing goal position".format(self.name)) 
 
                    point = self.point_locations.pop(0)  
                    goal_point = point   # goal is [x, y, z, theta]
                    response = self.set_goal(goal_point)  
                    time.sleep(0.5)  
 
                    if i == 0 and self.on == False:  
                        response = self.set_pump_proxy(True)  
                        time.sleep(0.5)
                    


                    time.sleep(2)  
                    rospy.loginfo("goal set successfully")                      
                    # if the point is last, give the robot pose with some negative offset
                    self.goal_xyz = goal_point[0:3]        
                    print('self.goal_xyz', self.goal_xyz)    
                    threshold = 0.06 # threshold can be changed      


                    distance = math.dist(self.goal_xyz, self.ee_pose) 
                    while distance > threshold:
                        try:
                            # Your loop code here
                            distance = math.dist(self.goal_xyz, self.ee_pose) 
                            print(distance)  
                            print('approaching goal')
                            time.sleep(0.5)
                        except KeyboardInterrupt:  
                            print('Loop stopped by user') 
                            break 
                    print('goal reached')  
                    if i == 4 and self.on == True :  
                        response = self.set_pump_proxy(False)   
                        time.sleep(0.5)  

                return py_trees.common.Status.SUCCESS  
            else:    
                return py_trees.common.Status.RUNNING 
        except: 
            self.logger.debug(
                "  {}: (Error  in Move_to_points block) calling service /manage_objects/let_object".format(self.name)) 
            return py_trees.common.Status.FAILURE 

    def terminate(self, new_status):
        
        self.logger.debug("  %s [Move_to_points::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status)) 
        
            
    def EE_pose(self, data): 
        # end-efffeector pose 
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z] 
         
    def wrap_angle(self, angle):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )  

    def dead_reckoning(self, odom): 

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.robot_state = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, self.wrap_angle(yaw)])   
    
        

    # Define a signal handler for Ctrl+C
    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0) 
   

 
class Move_robot(py_trees.behaviour.Behaviour): 

    # success if distance between goal and end-effector is less than threshold  
    def __init__(self, name):
        super(Move_robot, self).__init__(name) 
        self.pub_move_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.sub_pose_EE = rospy.Subscriber('pose_EE', PoseStamped, self.EE_pose) 
        self.image_sub = rospy.Subscriber("/small_aruco_position", Float64MultiArray, self.aruco_position)
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.dead_reckoning)  
        self.a = 0 
        time.sleep(1)    
    def setup(self):
        self.i = 0 
        self.a = 0  
        self.detect = 0
        self.logger.debug("  %s [Move_robot::setup()]" % self.name)
        rospy.wait_for_service('goal_server') 
        rospy.wait_for_service('weight_server')  
        rospy.wait_for_service('aruco_server')   
        rospy.wait_for_service('task_server')   
        try:
            self.set_aruco = rospy.ServiceProxy('aruco_server', intervention)  
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.set_task = rospy.ServiceProxy('task_server', intervention)

            self.logger.debug( 
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [Move_robot::setup()]" % self.name)

    def initialise(self):
        self.a = 0
        self.goal_xyz = 0
        self.distance = 0 
        self.aruco_pose = 0 
        self.weight = [1.0, 1.0, 1000.0, 1000.0, 1000.0, 1000.0]     
        self.weight_arm_pose = [1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]      
        self.weight_arm_pose_2 = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]       
        self.logger.debug("  %s [Move_robot::initialise()]" % self.name) 
   
    def update(self):  
        try:  
            signal.signal(signal.SIGINT, self.signal_handler)
            if blackboard.place_point is not None:  
                response = self.set_weight(self.weight_arm_pose_2)    
                time.sleep(0.2)   
                self.ind = [1] 
                rospy.loginfo("Weight not set successfully")     
                # rospy.loginfo("  {}: Publishing goal position34".format(blackboard.aruco))
      
                next_point = blackboard.place_point

                goal_point = PoseStamped() 
                goal_point.pose.position.x = next_point[0] 
                goal_point.pose.position.y = next_point[1] 
                print('position',next_point)   
                self.pub_move_goal.publish(goal_point)  
         
                print('GOAL_POSITION', next_point)    
                rospy.loginfo("goal set successfully")   
                
                while not rospy.get_param('goal_pose_turtle'): 
                    try:
                        print(rospy.get_param('goal_pose_turtle'))   
                        print('approaching goal')
                               
                        time.sleep(1)  
                    except KeyboardInterrupt: 
                        print('Loop stopped by user')  
                        break  

                return py_trees.common.Status.SUCCESS   


            elif True: 
                for i in range(2):    
                    self.logger.debug("  {}: Publishing goal position".format(self.name)) 

                    if i == 0:
                        
                        time.sleep(2)    

                        response = self.set_weight(self.weight_arm_pose) 

                        time.sleep(0.2)    
                        threshold = 0.3
                        
                        print('goal changed')
                        goal_position = blackboard.front_point  
                        print('after frontier')
                        response = self.set_goal(goal_position)  
                        time.sleep(2)    
                        print('after setting goal')  
                        self.goal_xyz = goal_position[0:3] 
                        print('goal', self.goal_xyz) 
                        distance = math.dist(self.goal_xyz, self.ee_pose)
                        print('dist', distance) 
                        while distance > threshold: 
                            try:
                                # Your loop code here
                                distance = math.dist(self.goal_xyz, self.ee_pose) 
                                print(distance)  
                                print('approaching goal')
                                time.sleep(0.5)
                            except KeyboardInterrupt:
                                print('Loop stopped by user')  
                                break 

                    else:
                        response = self.set_weight(self.weight)  
                        time.sleep(4)    
                        self.ind = [1]   
                        rospy.loginfo("Weight set successfully") 
                        goal_position = blackboard.aruco_pos   # goal is [x, y, z, theta]   # this is working  
                        
                        print('GOAL_POSITION', goal_position)  
                        print('self.robot_state', self.robot_state)   
                          
                        response = self.set_goal(goal_position)   
                        time.sleep(2)      
                        
                        rospy.loginfo("goal set successfully")   
                        
                        self.goal_xyz = goal_position[0:3]       
                        
                        print('self.goal_xyz', self.goal_xyz)  
                        # distance between end-effector and goal point 
                        
                           
                        threshold = 0.3 # threshold can be changed    
                        
                        distance = math.dist(self.goal_xyz, self.ee_pose)
                        while distance > threshold: 
                            try:
                                # Your loop code here
                                distance = math.dist(self.goal_xyz, self.ee_pose)  
                                if distance < 0.3:      
                                    self.set_weight(self.weight_arm_pose_2) 
                                print(distance)   
                                print('approaching goal') 
                                time.sleep(1) 
                            except KeyboardInterrupt: 
                                print('Loop stopped by user') 
                                break   
            
 
                    if self.distance < threshold: 
                        print('goal reached')  
                return py_trees.common.Status.SUCCESS  
            else:   
                return py_trees.common.Status.RUNNING 
        except:  
            self.logger.debug(
                "  {}: (Error  in Move_robot block) calling service /manage_objects/let_object".format(self.name)) 
            return py_trees.common.Status.FAILURE 

    def terminate(self, new_status):
        self.logger.debug("  %s [Move_robot::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status)) 

        
            
    def EE_pose(self, data):  
        # end-efffeector pose  
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]    
 
    def aruco_position(self, aruco_msg):  
        self.aruco_pose = aruco_msg.data  
        blackboard.aruco = self.aruco_pose  
        x = blackboard.aruco 
        print('on black_board',x) 
 

    def wrap_angle(self, angle):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) ) 
    
    def dead_reckoning(self, odom):  

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.robot_state = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, self.wrap_angle(yaw)]) 

        front_distance = 0.3  
        front_side_theta = self.robot_state[2] 
        front_point_x = self.robot_state[0] + front_distance * math.cos(front_side_theta)
        front_point_y = self.robot_state[1] + front_distance * math.sin(front_side_theta)  
        self.front_point = [front_point_x, front_point_y, -0.3, 0.0] 
        if self.a == 0:
            print('front_point', self.front_point)  
            blackboard.front_point = self.front_point      
            self.a = 1 

 
        # Define a signal handler for Ctrl+C
    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

    
        



class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Planning, self).__init__(name)
        self.pub_move_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) 
        self.image_sub = rospy.Subscriber("/small_aruco_position", Float64MultiArray, self.aruco_detected)  
        self.odom_sub = rospy.Subscriber("kobuki/odom", Odometry, self.get_odom)      
        time.sleep(1) 

    def setup(self):
        self.new_pose = 0
        blackboard.place_point = None  
        self.angle = float(math.radians(0))       
        self.possible_locations = [[0.0, -1.7], [2.5, 1.5], [3.0, 3.0, -0.3, self.angle], [4.0, 4.0, -0.3, self.angle], [5.0, 5.0, -0.3, self.angle]]   # [2.0, 0.3] for circuit2_with_arucos
        self.logger.debug("  %s [Get Position::setup()]" % self.name)                     
 
    def initialise(self):  
        self.aruco_pose = 0 
        self.logger.debug("  %s [Get Position::initialise()]" % self.name)    
 
    def update(self): 
        try:
            signal.signal(signal.SIGINT, self.signal_handler) 
            self.new_pose = self.new_pose + 1 
        
            self.logger.debug("  {}: Saving first position to blackboard".format(self.name))   
            if self.new_pose == 2 :
                position = self.possible_locations.pop(0)      
                    
                
                blackboard.place_point = position
                print('position',position)   
                time.sleep(1) 
                 
            elif self.new_pose == 1:
                rospy.set_param('goal_pose_turtle', 0) 
    
                while not rospy.get_param('goal_pose_turtle'): 
                    try:
                        print(rospy.get_param('goal_pose_turtle'))   
                        print('approaching goal')
                        
                        time.sleep(1)  
                    except KeyboardInterrupt:
                        print('Loop stopped by user')  
                        break   
            
            if True:   
                print('Goal set')  
                return py_trees.common.Status.SUCCESS 

            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error in the Get_Position block".format(self.name)) 
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Get Position::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status)) 
        
    def aruco_detected(self, aruco_point):  
        
        self.aruco_pose = aruco_point.data 
        
        blackboard.aruco_pos = self.aruco_pose   
        x = blackboard.aruco_pos   
        print('on black_board',x)   

    def get_odom(self, odom):
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])   

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)






        





if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboard = py_trees.blackboard.Blackboard()

    rospy.init_node("behavior_trees")
    
    def create_tree(): 
        move_robot = Move_robot("Move_robot")
        move_to_points = Move_to_points("Move_to_points")
        planning = Planning("planning")  

        root = py_trees.composites.Sequence(name="Planning", memory=True)  
        root.add_children([planning, move_robot, move_to_points])     

        return root 
    
    def run(it=2):                      
        root = create_tree()
        py_trees.display.render_dot_tree(root)   
        try:
            print("Call setup for all tree children") 
            root.setup_with_descendants()  
            print("Setup done!\n\n")
            py_trees.display.ascii_tree(root)
            
            for _ in range(it):
                root.tick_once() 
                time.sleep(1)     
        except KeyboardInterrupt: 
            pass

    run() 
