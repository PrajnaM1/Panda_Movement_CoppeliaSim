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


class action_server():

    goal_action = ''
    feedback = robot_geoFeedback()
    result = robot_geoResult()

    #WIPE PARAMETERS
    sel_x = 1.0
    sel_y = 1.0
    sel_z = 1.0

    force_x = 0.0
    force_y = 0.0
    force_z = 0.0
    torque_x = 0.0
    torque_y = 0.0
    torque_z = 0.0

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("wipe_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        
        #Subscribe to /move_status from move_to_contact_prim.py
        self.move_status_sub = rospy.Subscriber("/move_status", Bool, self.wipe, queue_size = 1)
   

    def execute_cb(self, goal):

        if self.a_server.is_preempt_requested():
            success = False
            rospy.loginfo('Preempted')
            self.a_server.set_preempted()
            print("Preempted.")

        action_server.goal_action = goal.type

        action_server.sel_x = goal.selection.x
        action_server.sel_y = goal.selection.y
        action_server.sel_z = goal.selection.z

        action_server.force_x = goal.ee_wrench.force.x
        action_server.force_y = goal.ee_wrench.force.y
        action_server.force_z = goal.ee_wrench.force.z
        action_server.torque_x = goal.ee_wrench.torque.x
        action_server.torque_y = goal.ee_wrench.torque.y
        action_server.torque_z = goal.ee_wrench.torque.z
        
        print("Wipe server created")
        
        if action_server.goal_action == 'wipe':
            print('WIPE action to be executed')

    def wipe(self, msg):
        success = True
       
        selection_msg = geometry_msgs.msg.Vector3()
        wrench_msg = geometry_msgs.msg.Wrench()
        pose_msg = geometry_msgs.msg.PoseStamped()

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        if (msg.data == True and action_server.goal_action == 'wipe'):

            print("Object has to be WIPED.")
            
            #Publish the Selection Vector
            selection_msg.x = action_server.sel_x 
            selection_msg.y = action_server.sel_y 
            selection_msg.z = action_server.sel_z
            selection_pub.publish(selection_msg)

            #Publish Wrench
            wrench_msg.force.x = action_server.force_x
            wrench_msg.force.y = action_server.force_y
            wrench_msg.force.z = action_server.force_z
            wrench_msg.torque.x = action_server.torque_x
            wrench_msg.torque.y = action_server.torque_y
            wrench_msg.torque.z = action_server.torque_z
            wrench_pub.publish(wrench_msg)

            #Publish the movement
            (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
            current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
            time.sleep(0.5)

            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = current_pose[0]
            pose_msg.pose.position.y = current_pose[1] + 0.2
            pose_msg.pose.position.z = current_pose[2]
            pose_msg.pose.orientation.x = current_pose[3]
            pose_msg.pose.orientation.y = current_pose[4]
            pose_msg.pose.orientation.z = current_pose[5]
            pose_msg.pose.orientation.w = current_pose[6]
            pose_pub.publish(pose_msg)
            time.sleep(5)

            (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
            current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
            time.sleep(0.5)

            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = current_pose[0]
            pose_msg.pose.position.y = current_pose[1] - 0.2
            pose_msg.pose.position.z = current_pose[2]
            pose_msg.pose.orientation.x = current_pose[3]
            pose_msg.pose.orientation.y = current_pose[4]
            pose_msg.pose.orientation.z = current_pose[5]
            pose_msg.pose.orientation.w = current_pose[6]
            pose_pub.publish(pose_msg)

            #Stop force application
            wrench_msg.force.x = 0.0
            wrench_msg.force.y = 0.0
            wrench_msg.force.z = 0.0
            wrench_msg.torque.x = 0.0
            wrench_msg.torque.y = 0.0
            wrench_msg.torque.z = 0.0
            wrench_pub.publish(wrench_msg)

                
            print('Wiping Complete!')

            action_server.feedback.movement = 'Movement and Wiping Complete!'
            self.a_server.publish_feedback(action_server.feedback)
            time.sleep(1)
        
            rate.sleep() 

if __name__=="__main__":

    rospy.init_node('action_server')

    pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)
    selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
    wrench_pub = rospy.Publisher('/panda/ee_wrench_goals', geometry_msgs.msg.Wrench, queue_size=1)
   
    w = action_server()

    rospy.spin()

