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
    gripper_content = False
    obj_reach = True
    feedback = robot_geoFeedback()
    result = robot_geoResult()

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("place_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        
        #Subscribe to /move_status from move_to_goal_prim.py
        self.move_status_sub = rospy.Subscriber("/move_status", Bool, self.release, queue_size = 1)
   

    def execute_cb(self, goal):

        if self.a_server.is_preempt_requested():
            success = False
            rospy.loginfo('Preempted')
            self.a_server.set_preempted()
            print("Preempted.")

        action_server.goal_action = goal.type
        action_server.gripper_content = goal.gripper_content
        action_server.obj_reach = goal.obj_reach

        print("Place server created")
        
        if action_server.goal_action == 'place':
            print('PLACE action to be executed')


    def release(self, msg):

        success = True

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 
                
        if (msg.data == True and action_server.goal_action == 'place'):
            if (action_server.gripper_content and action_server.obj_reach):
                print("Object has to be RELEASED.")
                subaction_pub.publish("release")
                time.sleep(2)
            else:
                if (not action_server.gripper_content):
                    print("Gripper is NOT holding an object!")
                if (not (action_server.obj_reach)):
                    print("Object is unreachable! Please check the reachability of the object.")

            
        action_server.feedback.movement = 'Object PLACED!'
        self.a_server.publish_feedback(action_server.feedback)

        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print("Current pose is: ", current_pose)

        if success:
            #Result = final end_effector coordinates
            action_server.result.final_coord.pose.position.x = trans[0]
            action_server.result.final_coord.pose.position.y = trans[1]
            action_server.result.final_coord.pose.position.z = trans[2]
            action_server.result.final_coord.pose.orientation.x = rot[0]
            action_server.result.final_coord.pose.orientation.y = rot[1]
            action_server.result.final_coord.pose.orientation.z = rot[2]
            action_server.result.final_coord.pose.orientation.w = rot[3]
            self.a_server.set_succeeded(action_server.result)

        rate.sleep() 

if __name__=="__main__":

    rospy.init_node('place_server')

    #Publish GRASP to simulator
    subaction_pub = rospy.Publisher("/subaction", String, queue_size = 1)

    pl = action_server()

    rospy.spin()

