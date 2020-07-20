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

    success = True

    feedback = robot_geoFeedback()
    result = robot_geoResult()
    move_req = False
    goal_action = ''
    sel_x = 1.0
    sel_y = 1.0
    sel_z = 1.0
    count = 0
    rf = 0

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("move_to_contact_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()

        #Subscribe to /panda/wrench 
        self.move_sub = rospy.Subscriber("/panda/wrench", geometry_msgs.msg.Wrench, self.move_to_contact, queue_size = 1)

    def execute_cb(self, goal):

        if self.a_server.is_preempt_requested():
            action_server.success = False
            rospy.loginfo('Preempted')
            self.a_server.set_preempted()
            print("Preempted.")

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        
        action_server.goal_action = goal.type
        action_server.move_req = goal.move
        action_server.sel_x = goal.selection.x
        action_server.sel_y = goal.selection.y
        action_server.sel_z = goal.selection.z

        print("Move to Contact server created")

        goal_pose = [round(goal.coord.pose.position.x,3), round(goal.coord.pose.position.y,3), round(goal.coord.pose.position.z,3), round(goal.coord.pose.orientation.x,3), round(goal.coord.pose.orientation.y,3), round(goal.coord.pose.orientation.z,3), round(goal.coord.pose.orientation.w,3)]
        
        diff = abs(round((current_pose[3] - goal_pose[3] + current_pose[4] - goal_pose[4] + current_pose[5] - goal_pose[5] + current_pose[6] - goal_pose[6]), 1))
        
        if action_server.goal_action == 'apply_force' and diff != 0.0:
    
            #ANGULAR MOVEMENT AS SPECIFIED IN THE 'GOAL'
            key_rots = R.from_quat([ [current_pose[3], current_pose[4], current_pose[5], current_pose[6]], [goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]]])
            key_times = [0, 1]

            slerp = Slerp(key_times, key_rots)

            times = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
            interp_rots = slerp(times)

            quat = interp_rots.as_quat()

            pose_msg = geometry_msgs.msg.PoseStamped()

            #Publish the point to ee_pose_goals
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = current_pose[0]
            pose_msg.pose.position.y = current_pose[1]
            pose_msg.pose.position.z = current_pose[2]
            pose_msg.pose.orientation.x = round(quat[6][0], 3)
            pose_msg.pose.orientation.y = round(quat[6][1], 3)
            pose_msg.pose.orientation.z = round(quat[6][2], 3)
            pose_msg.pose.orientation.w = round(quat[6][3], 3)
        

    def move_to_contact(self, msg):

        force_threshold = 45.0

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        pose_msg = geometry_msgs.msg.PoseStamped()

        move_contact_status = False

        #current_pose from tf
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print("function: Current pose is: ", current_pose)

        action_server.feedback.movement = 'Current Pose is: ' + str(current_pose)
        self.a_server.publish_feedback(action_server.feedback)
        
        if (action_server.move_req == False):

            #MOVE ALONG THIS DIRECTION TOWARDS CONTACT
            forces = np.sqrt(msg.force.x**2 + msg.force.y**2 + msg.force.z**2)
            print('CURRENT force: ', forces)
            action_server.rf = action_server.rf + forces
            print('RADIAL Force is: ', action_server.rf)
            action_server.count += 1
            print('COUNT: ', action_server.count)
            if (abs(round(action_server.rf,1)) < 18.8 and action_server.count == 1): #move

                action_server.feedback.movement = 'Moving towards contact..'
                self.a_server.publish_feedback(action_server.feedback)
                #time.sleep(0.5)

                #Status of Movement Completion
                move_contact_status = False

                if self.a_server.is_preempt_requested():
                    action_server.success = False
                    rospy.loginfo('Preempted')
                    #self.a_server.set_preempted()

                action_server.rf = 0 
                action_server.count = 0   

                #Publish the point to ee_pose_goals
                pose_msg.header.stamp = rospy.Time.now()
                if (action_server.sel_x == 0):
                    pose_msg.pose.position.x = current_pose[0] + 0.05
                else:
                    pose_msg.pose.position.x = current_pose[0]
                if (action_server.sel_y == 0):
                    pose_msg.pose.position.y = current_pose[1] + 0.05
                else:
                    pose_msg.pose.position.y = current_pose[1]
                if (action_server.sel_z == 0):
                    pose_msg.pose.position.z = current_pose[2] - 0.05
                else:
                    pose_msg.pose.position.z = current_pose[2]
                pose_msg.pose.orientation.x = current_pose[3]
                pose_msg.pose.orientation.y = current_pose[4]
                pose_msg.pose.orientation.z = current_pose[5]
                pose_msg.pose.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)
                print("Moving to point: ", pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w )
                #time.sleep(0.05)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                #print("Current pose is:", current_pose)
                action_server.feedback.movement = 'Current Pose is: ' + str(current_pose)
                self.a_server.publish_feedback(action_server.feedback)
                #time.sleep(0.5)

                
            elif (action_server.count != 1): 
                move_contact_status = False

            else:
                move_contact_status = True

            if (move_contact_status):

                print('Movement to Contact Complete!')
           
                action_server.feedback.movement = 'Movement to Contact Complete!'
                self.a_server.publish_feedback(action_server.feedback)
                #time.sleep(0.5)
            
                #Publish to higher level server that movement is complete
                move_contact_status_pub.publish(move_contact_status)
            
            rate.sleep() 

if __name__=="__main__":

    rospy.init_node('move_to_goal_server')

    #Publish to ee_path_goals to MOVE arm 
    pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)

    #Publish movement completion status to server
    move_contact_status_pub = rospy.Publisher('/move_contact_status', Bool, queue_size=1)
   
    mc = action_server()

    rospy.spin()

