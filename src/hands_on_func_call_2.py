#!/usr/bin/env python

# general libraries 
import numpy as np  
import math 
  
# importing the functions 
from mob_hands_on_func_define import *     

class Manipulator:   
    def __init__(self, theta): 
        self.theta = theta   
        self.revolute = [True, False, True, True , True , True]  
        self.dof = len(self.revolute)
        self.r = 0.0
        self.eta = np.zeros((3,1)) 
        self.T = np.zeros((4,4))  
        self.range = 0 
        # self.link = 0
 
    '''
        Method that updates the state of the robot. 

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time  
    '''
    
    def update(self, dq, dt, state):  # dq is a 6 by 1 vector, 1st is angular velocity and 2nd is linear velocity

        self.theta += (dq[2:, 0]).reshape(-1,1) * dt  

        self.eta[2,0] = state[2] 
        self.eta[0,0] = state[0]
        self.eta[1,0] = state[1] 
        
        # Base kinematics  
        Tb = np.array([[math.cos(self.eta[2,0]), -math.sin(self.eta[2,0]), 0, self.eta[0,0]],
                       [math.sin(self.eta[2,0]), math.cos(self.eta[2,0]), 0, self.eta[1,0]], 
                       [0,0,1,0], 
                       [0,0,0,1]])   
        
        self.T = kinematics_tb(self.theta, Tb)      
  
    '''
        Method that returns the end-effector Jacobian.
    ''' 
    def getEEJacobian(self, link): 
        return Jacobian(self.theta, self.eta[2,0], self.eta[0,0], link) # 4 is a link            

    ''' 
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]   

    '''
        Method that returns the position of a selected joint. 

        Argument:
        joint (integer): index of the joint 

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):  # joint is a link, if 3 is given then it is first joint of manipulator
        return self.theta[joint-3]          
    
    '''
        Method that returns base of the robot.  
    '''

    def getBasePose(self):
        return self.eta 
    
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    

    
    '''
        Method that returns transformation of a selected link. 
    '''
    def get_Se_LTransform(self, link):
        return self.T[link-1]     
 
'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired):
        self.name = name        # task title
        self.sigma_d = desired  # desired sigma 
        self.mobi_base = None  
        self.active = False
        self.a = 0 
 
        
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass

    def bool_is_Active(self): 
        return self.active  # self.active will be either true or false

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err 
    
        '''
        Method returning the mobile base position.
    ''' 
    def get_mobi_base(self):
        return self.mobi_base 
    
        '''
        Method returning the mobile base position.
    ''' 
    def get_eep(self):
        return self.eep  
 
'''
    Subclass of Task, representing the 2D position task. 
'''
## 2D position refers to the (x,y) coordinates of the end effector in a 2D plane

class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        self.J = np.zeros((3, self.link))

        self.err = np.zeros((3,1)) 
        self.active = True  
                
    def update(self, robot):  
        self.J = robot.getEEJacobian(self.link)[0:3]          # Update task Jacobian 

        X = (robot.getEETransform()[0:3, 3]).reshape(3,1)  
    
        self.err = ((self.getDesired()).reshape(3,1) - X).reshape(3,1)       # Update task error   
'''
    Subclass of Task, representing the 2D orientation task.   
'''

## 2D orientation refers to the angle between the end effector and the horizontal axis in the same 2D plane. It is usually measured in radians.

class Orientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link    # 4 for second joint 
        self.J = np.zeros((1,6))   
        self.err = np.zeros((1))  
        self.active = True 

    def update(self, robot):
        
        self.J = (robot.getEEJacobian(self.link)[-1]).reshape(1, self.link)   # last row of the jacobian   
        Y = robot.get_Se_LTransform(self.link)    
        orien = np.arctan2(Y[1,0], Y[0,0])  
        self.err = self.getDesired() - orien  

class BaseOrientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link   
        self.J = np.zeros((1, self.link)) 
        self.err = 0
        self.active = True 

    def update(self, robot):
        self.J = (robot.getEEJacobian(self.link)[-1]).reshape(1,6)   # last row of the jacobian   
        Y = robot.getEETransform()
        orien = np.arctan2(Y[1,0], Y[0,0]) 
        self.err = self.getDesired() - orien   
        
''' 
    Subclass of Task, representing the 2D configuration task.  
''' 

## 2D configuration refers to the combination of the 2D position and 2D orientation of the end effector.

class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        self.J = np.zeros((4, self.link)) 
        self.config = np.zeros((4,1))  
        self.active = True 
        self.err = np.zeros((4,1))   
    def update(self, robot):
        # Jacobian 
        self.J[0:3,:] = robot.getEEJacobian(self.link)[0:3] 
        self.J[-1,:] = robot.getEEJacobian(self.link)[-1].reshape(1,6)   

        transf = robot.getEETransform() 
        eep = transf[0:3,3].reshape(3,1)
        orien = np.arctan2(transf[1,0], transf[0,0])   
        self.config = np.vstack([eep, orien])  
        self.err = self.getDesired() - self.config  
        # pass # to remove

class Jointlimits3D(Task):  
    def __init__(self, name, desired, activation, link): 
        super().__init__(name, desired)
        self.activation = activation 
        self.link = link   
        self.J = np.zeros((1,6))    
        self.a = 0
        # self.J = np.zeros((1,5))  
        self.err = np.zeros((1))  
 
    # wraps the angle between -pi to +pi 
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def update(self, robot):     

        self.J = (robot.getEEJacobian(self.link)[5,:]).reshape(1,6)  
        # print('jac',self.J)   
        
        link_transform = robot.get_Se_LTransform(self.link)     
        
        orien = np.arctan2(link_transform[1,2], link_transform[0,2])    
        print('angle', orien)     
        print('joint angle ', robot.getJointPos(4))    
        if self.a == 1 and orien > self.activation[2]: 
            self.a = 0   
            self.active = False  
            self.err = 0.0      
  
        if self.a == -1 and orien < self.activation[0]:   
            print('activated')
            self.a = 0 
            self.active = False 
            self.err = 0.0   

        if self.a== 0 and orien > self.activation[1]:  
            print('un-activated')  
            self.a = -1 
            self.active = True  
            self.err = -1.0   

        if self.a == 0 and orien < self.activation[3]:   
            self.a = 1 
            self.active = True      
            self.err = 1.0   


# joint position 
class JointPosition3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        self.J = np.zeros((1, 6)) 
        self.err = np.zeros((1)) 
        self.active = True  
    def update(self, robot):
        self.J = (robot.getEEJacobian(self.link)[5,:]).reshape(1,6) 
        sigma = robot.getJointPos(self.link) 
        print('sigma', sigma)  
        print('sigma_d', self.getDesired())   
 
        self.err = self.getDesired() - sigma

  