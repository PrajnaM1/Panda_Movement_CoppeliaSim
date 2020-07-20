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

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("move_to_goal_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()

    def execute_cb(self, goal):

        success = True

        #Status of Movement Completion
        move_status = False
        
        feedback = robot_geoFeedback()
    
        result = robot_geoResult()

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        pose_msg = geometry_msgs.msg.PoseStamped()

        #current_pose from tf
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print("Current pose is: ", current_pose)

        feedback.movement = 'Current Pose is: ' + str(current_pose)
        self.a_server.publish_feedback(feedback)

        #goal_pose from client
        goal_pose = [round(goal.coord.pose.position.x,3), round(goal.coord.pose.position.y,3), round(goal.coord.pose.position.z,3), round(goal.coord.pose.orientation.x,3), round(goal.coord.pose.orientation.y,3), round(goal.coord.pose.orientation.z,3), round(goal.coord.pose.orientation.w,3)]
        
        if (goal.move == True):

            feedback.movement = 'Movement Required!'
            self.a_server.publish_feedback(feedback)
            time.sleep(0.5)

            feedback.movement = 'Goal Pose is: ' + str(goal_pose)
            self.a_server.publish_feedback(feedback)
            #time.sleep(0.5)

            x_diff = round((goal_pose[0] - current_pose[0]),3)
            y_diff = round((goal_pose[1] - current_pose[1]),3)
            z_diff = round((goal_pose[2] - current_pose[2]),3)

            x_ang_diff = round((goal_pose[3] - current_pose[3]),3)
            y_ang_diff = round((goal_pose[4] - current_pose[4]),3)
            z_ang_diff = round((goal_pose[5] - current_pose[5]),3)
            w_ang_diff = round((goal_pose[6] - current_pose[6]),3)

            while ( (abs(x_diff) > 0.015)  or (abs(y_diff) > 0.015) or (abs(z_diff) > 0.015)):

                if self.a_server.is_preempt_requested():
                    success = False
                    rospy.loginfo('Preempted')
                    self.a_server.set_preempted()
                    break 
                    
                #LINEAR
                r = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
                
                if abs(r) >= 0.1:
                    step_size = 0.05
                else:
                    step_size = 0.01

                num_pts = abs(int((r // step_size) + 1))

                if (abs(x_diff) > 0.015):
                    x_step = x_diff / num_pts
                    x = np.arange(current_pose[0], goal_pose[0], x_step)
                else: 
                    print("x_diff nearly 0")
                    x = np.full(num_pts, current_pose[0])

                if (abs(y_diff) > 0.015):
                    y_step = y_diff / num_pts
                    y = np.arange(current_pose[1], goal_pose[1], y_step)
                else: 
                    print("y_diff nearly 0")
                    y = np.full(num_pts, current_pose[1])

                if (abs(z_diff) > 0.015):
                    z_step = z_diff / num_pts
                    z = np.arange(current_pose[2], goal_pose[2], z_step)
                else: 
                    print("z_diff nearly 0")
                    z = np.full(num_pts, current_pose[2])
                
                #ANGULAR
                key_rots = R.from_quat([ [current_pose[3], current_pose[4], current_pose[5], current_pose[6]], [goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]]])
                key_times = [0, 1]

                slerp = Slerp(key_times, key_rots)

                times = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
                interp_rots = slerp(times)

                quat = interp_rots.as_quat()

                #Publish the point to ee_pose_goals
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = round (x[1], 3)
                pose_msg.pose.position.y = round (y[1], 3)
                pose_msg.pose.position.z = round (z[1], 3)
                pose_msg.pose.orientation.x = round(quat[6][0], 3)
                pose_msg.pose.orientation.y = round(quat[6][1], 3)
                pose_msg.pose.orientation.z = round(quat[6][2], 3)
                pose_msg.pose.orientation.w = round(quat[6][3], 3)

                pose_pub.publish(pose_msg)
                print("Moving to point: ", pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w )
                time.sleep(0.05)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                print("Current pose is:", current_pose)
                feedback.movement = 'Current Pose is: ' + str(current_pose)
                self.a_server.publish_feedback(feedback)
                #time.sleep(0.5)
                

                x_diff = round((goal_pose[0] - current_pose[0]),3)
                y_diff = round((goal_pose[1] - current_pose[1]),3)
                z_diff = round((goal_pose[2] - current_pose[2]),3)

                x_ang_diff = round((goal_pose[3] - current_pose[3]),3)
                y_ang_diff = round((goal_pose[4] - current_pose[4]),3)
                z_ang_diff = round((goal_pose[5] - current_pose[5]),3)
                w_ang_diff = round((goal_pose[6] - current_pose[6]),3)

                
            move_status = True
            print('Movement Complete!')
           
            feedback.movement = 'Movement Complete!'
            self.a_server.publish_feedback(feedback)
            #time.sleep(0.5)
            
            #Publish to higher level server that movement is complete
            move_status_pub.publish(move_status)
            '''
            #Result = final end_effector coordinates
            result.final_coord.pose.position.x = trans[0]
            result.final_coord.pose.position.y = trans[1]
            result.final_coord.pose.position.z = trans[2]
            result.final_coord.pose.orientation.x = rot[0]
            result.final_coord.pose.orientation.y = rot[1]
            result.final_coord.pose.orientation.z = rot[2]
            result.final_coord.pose.orientation.w = rot[3]
            self.a_server.set_succeeded(result)
            '''
            rate.sleep() 

if __name__=="__main__":

    rospy.init_node('move_to_goal_server')

    #Publish to ee_path_goals to MOVE arm 
    pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)

    #Publish movement completion status to server
    move_status_pub = rospy.Publisher('/move_status', Bool, queue_size=1)
   
    m = action_server()

    rospy.spin()

