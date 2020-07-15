#! /usr/bin/env python

import rospy
import actionlib
from demo.msg import robot_geoAction, robot_geoFeedback, robot_geoResult
import math
import tf
import PyKDL 
import tf2_ros
import geometry_msgs.msg
from tf.transformations import *
import numpy as np
from numpy.linalg import norm
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

#IMPORTANT: This is NOT an action server

global subaction
global joint
joint = JointState()

def subaction(msg):
    
    subaction = msg.data
    if subaction != "grasp" and subaction != "release":
        print("Invalid primitive!")
    print('Subaction is: ', subaction)
    action_pub.publish(subaction)

'''
def state(msg):
    
    #Get the current joint states
    joint.header.stamp = rospy.Time.now()
    joint.name = ["panda_joint1", "panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"]
    joint.position = list(msg.position)
    
    if subaction == "grasp":
        print("Grasping...")
        joint.position[-1] = 0.0
        joint.position[-2] = 0.0
        state_pub.publish(joint)


    if subaction == "release":
        joint.position[-1] = 0.035
        joint.position[-2] = 0.035
        state_pub.publish(joint)

'''
if __name__=="__main__":

    rospy.init_node('grasp_release_server')

    #Subscriber to PICK and PLACE nodes
    subaction_sub = rospy.Subscriber("/subaction", String, subaction, queue_size = 1)

    '''
    #Subscriber to /panda/joint_states
    state_sub = rospy.Subscriber("/panda/joint_states", JointState, state, queue_size = 1)
    
    #Publish new states to /panda/joint_states
    state_pub = rospy.Publisher("/panda/joint_states", JointState, queue_size = 1)
    '''

    #Publish grasp/release to /panda/commands topic
    action_pub = rospy.Publisher("/panda/commands", String, queue_size = 1)

    rospy.spin()
