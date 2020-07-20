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

    #SCREW PARAMETERS
    depth = 0.0
    lead = 0.0

    sel_x = 1.0
    sel_y = 1.0
    sel_z = 0.0

    force_x = 0.0
    force_y = 0.0
    force_z = 0.0
    torque_x = 0.0
    torque_y = 0.0
    torque_z = 0.0
    
    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("screw_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        
        #Subscribe to /move_status from move_to_goal_prim.py
        self.move_status_sub = rospy.Subscriber("/move_status", Bool, self.screw, queue_size = 1)
   

    def execute_cb(self, goal):

        if self.a_server.is_preempt_requested():
            success = False
            rospy.loginfo('Preempted')
            self.a_server.set_preempted()
            print("Preempted.")

        action_server.goal_action = goal.type

        action_server.depth = goal.screw_depth
        action_server.lead = goal.screw_lead

        action_server.sel_x = goal.selection.x
        action_server.sel_y = goal.selection.y
        action_server.sel_z = goal.selection.z

        action_server.force_x = goal.ee_wrench.force.x
        action_server.force_y = goal.ee_wrench.force.y
        action_server.force_z = goal.ee_wrench.force.z
        action_server.torque_x = goal.ee_wrench.torque.x
        action_server.torque_y = goal.ee_wrench.torque.y
        action_server.torque_z = goal.ee_wrench.torque.z
        
        print("Screw server created")
        
        if action_server.goal_action == 'screw':
            print('SCREW action to be executed')


    def screw(self, msg):

        success = True

        selection_msg = geometry_msgs.msg.Vector3()
        wrench_msg = geometry_msgs.msg.Wrench()
        pose_msg = geometry_msgs.msg.PoseStamped()

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        if (msg.data == True and action_server.goal_action == 'screw'):
            print("Object has to be SCREWED.")
            num_rotations = action_server.depth // action_server.lead

            #Publish the Selection Vector
            selection_msg.x = action_server.sel_x 
            selection_msg.y = action_server.sel_y 
            selection_msg.z = action_server.sel_z
            selection_pub.publish(selection_msg)

            
            for i in range(2*num_rotations):
                #SCREW one round (180 deg) with z-force of -5N and come back up

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                
                wrench_msg.force.x = action_server.force_x
                wrench_msg.force.y = action_server.force_y
                wrench_msg.force.z = action_server.force_z
                wrench_msg.torque.x = action_server.torque_x
                wrench_msg.torque.y = action_server.torque_y
                wrench_msg.torque.z = action_server.torque_z
                wrench_pub.publish(wrench_msg)

                #180 degrees movement 
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = current_pose[0]
                pose_msg.pose.position.y = current_pose[1] 
                pose_msg.pose.position.z = current_pose[2] 
                pose_msg.pose.orientation.x = current_pose[3]
                pose_msg.pose.orientation.y = current_pose[4]
                pose_msg.pose.orientation.z = current_pose[5] - 0.5
                pose_msg.pose.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)

                #STOP applying FORCE
                wrench_msg.force.x = 0.0
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = 0.0
                wrench_msg.torque.x = 0.0
                wrench_msg.torque.y = 0.0
                wrench_msg.torque.z = 0.0
                wrench_pub.publish(wrench_msg)

                #Go back to previous position
                pose_msg.header.stamp = rospy.Time.now() 
                pose_msg.pose.position.x = current_pose[0]
                pose_msg.pose.position.y = current_pose[1] 
                pose_msg.pose.position.z = current_pose[2] 
                pose_msg.pose.orientation.x = current_pose[3]
                pose_msg.pose.orientation.y = current_pose[4]
                pose_msg.pose.orientation.z = current_pose[5]
                pose_msg.pose.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)


            '''
            for i in range(2*num_rotations):
                #SCREW one round (180 deg) with torque of -5N and come back up

                #Publish Wrench (TORQUE) for 180 degrees turn
                wrench_msg.force.x = action_server.force_x
                wrench_msg.force.y = action_server.force_y
                wrench_msg.force.z = action_server.force_z
                wrench_msg.torque.x = action_server.torque_x
                wrench_msg.torque.y = action_server.torque_y
                wrench_msg.torque.z = action_server.torque_z
                wrench_pub.publish(wrench_msg)
                time.sleep(5)

                #Stop the rotation
                wrench_msg.force.x = 0.0
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = 0.0
                wrench_msg.torque.x = 0.0
                wrench_msg.torque.y = 0.0
                wrench_msg.torque.z = 0.0
                wrench_pub.publish(wrench_msg)
                time.sleep(1)

                #Move arm up
                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                time.sleep(0.5)

                pose_msg.position.x = current_pose[0]
                pose_msg.position.y = current_pose[1] 
                pose_msg.position.z = current_pose[2] + 0.2
                pose_msg.orientation.x = current_pose[3]
                pose_msg.orientation.y = current_pose[4]
                pose_msg.orientation.z = current_pose[5]
                pose_msg.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)
                time.sleep(0.5)

                #Rotate back
                wrench_msg.force.x = action_server.force_x
                wrench_msg.force.y = action_server.force_y
                wrench_msg.force.z = action_server.force_z
                wrench_msg.torque.x = action_server.torque_x
                wrench_msg.torque.y = action_server.torque_y
                wrench_msg.torque.z = -action_server.torque_z
                wrench_pub.publish(wrench_msg)
                time.sleep(5)
     
                #Move back to screw for next rotation
                pose_msg.position.x = current_pose[0]
                pose_msg.position.y = current_pose[1] 
                pose_msg.position.z = current_pose[2] - 0.2
                pose_msg.orientation.x = current_pose[3]
                pose_msg.orientation.y = current_pose[4]
                pose_msg.orientation.z = current_pose[5]
                pose_msg.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)
                time.sleep(0.5)
            '''   
        action_server.feedback.movement = 'SCREW action completed!'
        self.a_server.publish_feedback(action_server.feedback)

        #Move back to RESTING position
        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.5
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        pose_pub.publish(pose_msg)
        time.sleep(0.5)

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

    rospy.init_node('screw_server')

    torque_pub = rospy.Publisher("/panda/ee_wrench_goals", geometry_msgs.msg.Wrench, queue_size = 1)
    selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
    pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)

    sc = action_server()

    rospy.spin()

