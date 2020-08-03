#! /usr/bin/env python

import rospy 
import actionlib
from demo.msg import robot_geoAction, robot_geoGoal
from panda_ros_msgs.msg import HybridPose, JointPose
import time

#In this program, goal coordinates and type of action are specified and sent to the server for end effector movement

def feedback_cb(msg):
    print('Feedback received: ', msg)

def call_server():

    #SERVERS
    
    client = actionlib.SimpleActionClient('action_as', robot_geoAction)
    client.wait_for_server()
    
    goal = robot_geoGoal()

    #GOAL Coordinates
    goal.coord.pose.position.x = 0.5
    goal.coord.pose.position.y = 0.0
    goal.coord.pose.position.z = 0.1
    goal.coord.pose.orientation.x = 0.0
    goal.coord.pose.orientation.y = 0.0
    goal.coord.pose.orientation.z = 0.0
    goal.coord.pose.orientation.w = 1.0

    #GOAL TYPE
    goal.type = 'pick'

    #SCREW PARAMETERS
    goal.direction = 'clockwise'
    goal.screw_depth = 25.4 #1 inch
    goal.screw_lead = 1.5   #in mm

    #PICK AND PLACE PARAMETERS
    goal.obj_reach = True
    goal.gripper_content = False

    #APPLY_FORCE PARAMETERS
    #goal.force_axis = 'z'
    #goal.force_time = 1
    
    #Selection Vector (1- position, 0- wrench)
    goal.selection.x = 1.0
    goal.selection.y = 1.0
    goal.selection.z = 0.0

    #Wrench
    goal.ee_wrench.force.x = 0.0
    goal.ee_wrench.force.y = 0.0
    goal.ee_wrench.force.z = -5.0
    goal.ee_wrench.torque.x = 0.0
    goal.ee_wrench.torque.y = 0.0
    goal.ee_wrench.torque.z = 0.0

    client.send_goal(goal, feedback_cb=feedback_cb)
    
    client.wait_for_result()
    result = client.get_result()
    
    return result                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      


if __name__=="__main__":

    try:
        rospy.init_node('action_client')
        
        result = call_server()                                             

        print('Final end_effector coordinates: ', result.final_coord.pose.position.x, result.final_coord.pose.position.y, result.final_coord.pose.position.z, result.final_coord.pose.orientation.x, result.final_coord.pose.orientation.y, result.final_coord.pose.orientation.z, result.final_coord.pose.orientation.w)

    except rospy.ROSInterruptException as e:
        print('Exception occured!', e)






