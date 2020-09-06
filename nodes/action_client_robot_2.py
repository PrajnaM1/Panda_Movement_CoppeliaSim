#! /usr/bin/env python

import rospy 
import actionlib
from demo.msg import robot_geoAction, robot_geoGoal
from panda_ros_msgs.msg import HybridPose, JointPose
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import actionlib
import time
import math

def feedback_cb(msg):
    print('Feedback received: ', msg)

def call_server(action):
    
    #Server
    client = actionlib.SimpleActionClient('action_as', robot_geoAction)
    client.wait_for_server()
    
    #GOAL Coordinates
    goal = robot_geoGoal()

    '''
    #PICK
    goal.coord.pose.position.x = 0.58
    goal.coord.pose.position.y = 0.00
    goal.coord.pose.position.z = 0.06
    goal.coord.pose.orientation.x = 0.0
    goal.coord.pose.orientation.y = 0.0
    goal.coord.pose.orientation.z = 0.0
    goal.coord.pose.orientation.w = 1.0
    '''
    '''
    #Move to contact
    goal.coord.pose.position.x = 0.50
    goal.coord.pose.position.y = 0.00
    goal.coord.pose.position.z = 0.03
    goal.coord.pose.orientation.x = 0.0
    goal.coord.pose.orientation.y = 0.0
    goal.coord.pose.orientation.z = 0.0
    goal.coord.pose.orientation.w = 1.0
    '''
    '''
    #Screw Position
    goal.coord.pose.position.x = 0.54
    goal.coord.pose.position.y = 0.15
    goal.coord.pose.position.z = 0.07
    goal.coord.pose.orientation.x = 0.0
    goal.coord.pose.orientation.y = 0.0
    goal.coord.pose.orientation.z = 0.0
    goal.coord.pose.orientation.w = 1.0
    '''

    #Selection Vector (1- position, 0- wrench)
    goal.selection.x = 1.0
    goal.selection.y = 1.0
    goal.selection.z = 0.0

    #Wrench
    goal.ee_wrench.force.x = 0.0
    goal.ee_wrench.force.y = 0.0
    goal.ee_wrench.force.z = 5.0
    goal.ee_wrench.torque.x = 0.0
    goal.ee_wrench.torque.y = 0.0
    goal.ee_wrench.torque.z = 0.0

    if action == 'pick':

        goal.obj_reach = True
        goal.gripper_content = False
    
        if goal.obj_reach and not goal.gripper_content:

            #move_to_goal primitive
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.55
            goal.coord.pose.position.y = 0.20
            goal.coord.pose.position.z = 0.14
            goal.coord.pose.orientation.x = 0.5
            goal.coord.pose.orientation.y = -0.5
            goal.coord.pose.orientation.z = 1.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
    
            #Grasp
            goal.type = 'grasp'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Rest
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.50
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            
        elif not goal.obj_reach:
            print("Object NOT reachable!")

        elif goal.gripper_content:
            print("The gripper is already holding something!")
        
    elif action == 'move_to_goal':
        '''
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.57
        goal.coord.pose.position.y = -0.16
        goal.coord.pose.position.z = 0.15
        goal.coord.pose.orientation.x = 0.14
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.57
        goal.coord.pose.position.y = -0.16
        goal.coord.pose.position.z = 0.13
        goal.coord.pose.orientation.x = 0.14
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        '''
        '''
        #screw 90 deg
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.56
        goal.coord.pose.position.y = 0.20
        goal.coord.pose.position.z = 0.21
        goal.coord.pose.orientation.x = 1.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.56
        goal.coord.pose.position.y = 0.22
        goal.coord.pose.position.z = 0.21
        goal.coord.pose.orientation.x = 1.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        '''
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.60
        goal.coord.pose.position.y = 0.23
        goal.coord.pose.position.z = 0.20
        goal.coord.pose.orientation.x = 0.7068252
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 0.7073883
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

    elif action == 'place':

        goal.obj_reach = True
        goal.gripper_content = True

        if goal.obj_reach and goal.gripper_content:
            
            #move_to_goal primitive (box)
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.55
            goal.coord.pose.position.y = 0.40
            goal.coord.pose.position.z = 0.40
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            
            #Down
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.55
            goal.coord.pose.position.y = 0.40
            goal.coord.pose.position.z = 0.20
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
        
            
            #Release
            goal.type = 'release'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Up
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.55
            goal.coord.pose.position.y = 0.40
            goal.coord.pose.position.z = 0.40
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            
            #Rest
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.50
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()

        elif not goal.obj_reach:
            print("Object NOT reachable!")

        elif not goal.gripper_content:
            print("The gripper has no object to place!")

    elif action == 'apply_force':
        #move_to_goal primitive
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.50
        goal.coord.pose.position.y = 0.00
        goal.coord.pose.position.z = 0.10
        goal.coord.pose.orientation.x = 0.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        #move_to_contact primitive
        goal.type = 'move_to_contact'
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        
        #apply_force
        goal.type = 'apply_force'
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        
    elif action == 'screw':

        goal.direction = 'clockwise'
        goal.screw_depth = 25.4 #1 inch
        goal.screw_lead = 1.5   #in mm
        #num_rotations = goal.screw_depth // goal.screw_lead
        #total_rounds = num_rotation * 2
        #goal.rot_angle = math.pi/2 

        round = 0
        '''
        while round < 1:

            #Move to goal
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.20
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.1
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)

            #Grasp
            goal.type = 'grasp'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(2)
            
            #Rotate screw
            goal.type = 'rotate_screw'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.20
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.1
            goal.coord.pose.orientation.z = 0.0
            round += 1
            time.sleep(1)
            
            #Stop Force
            goal.type = 'stop_force'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Release
            goal.type = 'release'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Rotate back
            goal.type = 'rotate_back'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.20
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.1
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)

            #Rest
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.50
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
           
        '''
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.60
        goal.coord.pose.position.y = 0.23
        goal.coord.pose.position.z = 0.20
        goal.coord.pose.orientation.x = 0.7068252
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 0.7073883
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        time.sleep(1)

        while round < 1:

            #Grasp
            goal.type = 'grasp'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Rotate screw
            goal.type = 'rotate_screw'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            round += 1
            #time.sleep(1)
            
            #Stop Force
            goal.type = 'stop_force'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Release
            goal.type = 'release'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
            
            #Rotate back
            goal.type = 'rotate_back'
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            time.sleep(1)
             
        if round == 1:
            #Rest
            goal.type = 'move_to_goal'
            goal.coord.pose.position.x = 0.50
            goal.coord.pose.position.y = 0.00
            goal.coord.pose.position.z = 0.50
            goal.coord.pose.orientation.x = 0.0
            goal.coord.pose.orientation.y = 0.0
            goal.coord.pose.orientation.z = 0.0
            goal.coord.pose.orientation.w = 1.0
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
        
    elif action == "insert":
        
        goal.coord.pose.orientation.x = 0.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        #Grasp
        goal.type = 'grasp'
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        time.sleep(1)

        #Move to a point near hole
        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.70
        goal.coord.pose.position.y = 0.00
        goal.coord.pose.position.z = 0.15
        goal.coord.pose.orientation.x = 0.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()

        goal.type = 'move_to_goal'
        goal.coord.pose.position.x = 0.70
        goal.coord.pose.position.y = 0.00
        goal.coord.pose.position.z = 0.12
        goal.coord.pose.orientation.x = 0.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        
        goal.type = 'insert'
        goal.coord.pose.position.x = 0.71
        goal.coord.pose.position.y = 0.00
        goal.coord.pose.position.z = 0.085
        goal.coord.pose.orientation.x = 0.0
        goal.coord.pose.orientation.y = 0.0
        goal.coord.pose.orientation.z = 0.0
        goal.coord.pose.orientation.w = 1.0
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
        
    result = client.get_result()
    return result                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      


if __name__=="__main__":

    try:
        rospy.init_node('action_client')

        action = 'move_to_goal'
        result = call_server(action)                                             

        print('Final end_effector coordinates: ', result.final_coord.pose.position.x, result.final_coord.pose.position.y, result.final_coord.pose.position.z, result.final_coord.pose.orientation.x, result.final_coord.pose.orientation.y, result.final_coord.pose.orientation.z, result.final_coord.pose.orientation.w)

    except rospy.ROSInterruptException as e:
        print('Exception occured!', e)






