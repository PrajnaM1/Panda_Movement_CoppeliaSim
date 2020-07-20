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

if __name__=="__main__":

    rospy.init_node('screw_trial')

    wrench_pub = rospy.Publisher("/panda/ee_wrench_goals", geometry_msgs.msg.Wrench, queue_size = 1)
    selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
    pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)

    #SCREW PARAMETERS
    depth = 25.4
    lead = 1.5
    #num_rotations = depth // lead
    num_rotations = 1

    selection_msg = geometry_msgs.msg.Vector3()
    wrench_msg = geometry_msgs.msg.Wrench()
    pose_msg = geometry_msgs.msg.PoseStamped()

    selection_msg.x = 1.0
    selection_msg.y = 1.0 
    selection_msg.z = 0.0
    selection_pub.publish(selection_msg)
    
    #Listen to transforms
    listener = tf.TransformListener()
    rospy.sleep(1)

    rate = rospy.Rate(100.0) 

    print("Object has to be SCREWED.")

    (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
    current_pose_prev = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
    print("Starting Position ", current_pose_prev)   
    time.sleep(0.5)  
            
    for i in range(int(2*num_rotations)):
        
        #SCREW one round (180 deg) with z-force of -5N and come back up

        #STEP 2
        #print("Publishing Wrench..")
        wrench_msg.force.x = 0.0
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = -5.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0
        wrench_pub.publish(wrench_msg)
        time.sleep(0.5)

        #180 degrees movement for SCREW
        #print("Publishing 180 turn..")
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = current_pose_prev[0]
        pose_msg.pose.position.y = current_pose_prev[1] 
        pose_msg.pose.position.z = current_pose_prev[2] 
        pose_msg.pose.orientation.x = current_pose_prev[3]
        pose_msg.pose.orientation.y = current_pose_prev[4]
        pose_msg.pose.orientation.z = current_pose_prev[5] + 1.0
        pose_msg.pose.orientation.w = current_pose_prev[6]
        pose_pub.publish(pose_msg) 
        time.sleep(0.5)

        #STOP applying FORCE
        #print("Stopping force..")
        wrench_msg.force.x = 0.0
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0
        wrench_pub.publish(wrench_msg)
        time.sleep(0.5)
       
        #STEP 3
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print("After a round of SCREWing ", current_pose)

        #STEP 4: Go back UP to previous position 
        #print("Moving back up..")
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = current_pose_prev[0]
        pose_msg.pose.position.y = current_pose_prev[1] 
        pose_msg.pose.position.z = current_pose_prev[2]
        pose_msg.pose.orientation.x = current_pose[3]
        pose_msg.pose.orientation.y = current_pose[4]
        pose_msg.pose.orientation.z = current_pose[5]
        pose_msg.pose.orientation.w = current_pose[6]
        pose_pub.publish(pose_msg)
        print("Moved UP to: ", pose_msg)
        time.sleep(3)
        
        #Rotate back to previous position
        #print("Turning back..")
        pose_msg.header.stamp = rospy.Time.now() 
        pose_msg.pose.position.x = current_pose_prev[0]
        pose_msg.pose.position.y = current_pose_prev[1] 
        pose_msg.pose.position.z = current_pose_prev[2]
        pose_msg.pose.orientation.x = current_pose_prev[3]
        pose_msg.pose.orientation.y = current_pose_prev[4]
        pose_msg.pose.orientation.z = current_pose_prev[5]
        pose_msg.pose.orientation.w = current_pose_prev[6]
        pose_pub.publish(pose_msg)
        #print("Turned back to: ", pose_msg)
        time.sleep(3)

        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose_t = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        time.sleep(0.5)
        print("Moved back to: ", current_pose_t, " Note: Should match with prev/starting position")   

        #STEP 5
        pose_msg.header.stamp = rospy.Time.now() 
        pose_msg.pose.position.x = current_pose_prev[0]
        pose_msg.pose.position.y = current_pose_prev[1] 
        pose_msg.pose.position.z = current_pose[2]
        pose_msg.pose.orientation.x = current_pose_prev[3]
        pose_msg.pose.orientation.y = current_pose_prev[4]
        pose_msg.pose.orientation.z = current_pose_prev[5]
        pose_msg.pose.orientation.w = current_pose_prev[6]
        pose_pub.publish(pose_msg)
        time.sleep(3)
        print("Finally moving down to position: ", pose_msg.pose.position.z)

        #STEP 6
        current_pose_prev[0] = current_pose[0]
        current_pose_prev[1] = current_pose[1]
        current_pose_prev[2] = current_pose[2]
        #print("New current pose prev is: ", current_pose_prev)

    (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
    current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
    time.sleep(0.5)
    print("Finally at position: ", current_pose)   
    rospy.spin()

