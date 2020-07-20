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
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String

class action_server():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("arm_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()

    def execute_cb(self, goal):

        global joint_position

        success = True
        
        feedback = robot_geoFeedback()
    
        result = robot_geoResult()
        completion_status_lin = False
        completion_status_ang = False

        small_x = False
        small_y = False
        small_z = False

        #Listen to transforms
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(100.0) 

        #Initialization
        old_angles_x = 0.00
        old_angles_y = 0.00
        old_angles_z = 0.00

        moved_x = 0
        moved_y = 0
        moved_z = 0

        count=0

        goal_x_lin = round(goal.coord.pose.position.x,1)
        goal_y_lin = round(goal.coord.pose.position.y,1)
        goal_z_lin = round(goal.coord.pose.position.z,1)
        goal_x_ang = round(goal.coord.pose.orientation.x,1)
        goal_y_ang = round(goal.coord.pose.orientation.y,1)
        goal_z_ang = round(goal.coord.pose.orientation.z,1)
        goal_w_ang = round(goal.coord.pose.orientation.w,1)

        #Goal Coordinates
        feedback.movement = 'The goal coordinates are: ' + str(goal_x_lin) + ' ' + str(goal_y_lin) + ' ' + str(goal_z_lin) + str(goal_x_ang) + ' ' + str(goal_y_ang) + ' ' + str(goal_z_ang) + ' ' + str(goal_w_ang)
        self.a_server.publish_feedback(feedback)

        #Panda Movement
        while ((completion_status_lin == False) or (completion_status_ang == False)):

            if self.a_server.is_preempt_requested():
                success = False
                break 

            try:
                ee2b = tfBuffer.lookup_transform('panda_link0', 'sim_end_effector', rospy.Time())

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            #Initialization
            vel_msg = geometry_msgs.msg.Twist()

            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        
            ##Calculation of LINEAR Differences (goal = sim_end_effector)

            diff_x = -ee2b.transform.translation.x + goal_x_lin
            diff_y = -ee2b.transform.translation.y + goal_y_lin
            diff_z = -ee2b.transform.translation.z + goal_z_lin

            max_time_trans = 0.14
            r = np.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
            d_t = r / max_time_trans

            if ((round(diff_x,3) != 0.000) or (round(diff_y,3) != 0.000) or (round(diff_z,3) != 0.000)):
                vel_msg.linear.x = round((round(diff_x,3)/d_t), 3)
                vel_msg.linear.y = round((round(diff_y,3)/d_t), 3)
                vel_msg.linear.z = round((round(diff_z,3)/d_t), 3)
                feedback.movement = 'Moving with linear speeds along X,Y & Z: ' + str(vel_msg.linear.x) + ' ' + str(vel_msg.linear.y) + ' ' + str(vel_msg.linear.z)
                self.a_server.publish_feedback(feedback)
                
            else:
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0
                completion_status_lin = True
                feedback.movement = 'No Linear Movement!'
                self.a_server.publish_feedback(feedback)
            
            ##Calculation of ANGLES to be rotated (goal = end_effector)
            
            #Current Rotation Angles
            q_current = PyKDL.Rotation.Quaternion(ee2b.transform.rotation.x,ee2b.transform.rotation.y,ee2b.transform.rotation.z,ee2b.transform.rotation.w)
            current_angle = q_current.GetRPY()

            #Final Rotation Angles
            q_goal = PyKDL.Rotation.Quaternion(goal_x_ang, goal_y_ang, goal_z_ang, goal_w_ang)

            #Calculation of Angular Differences in Quaternion
            q = q_goal*q_current.Inverse()
            difference_angle = q.GetRPY()
            diff_angles  = list(difference_angle)
        
            #print('Difference angles:', diff_angles)
        
            #ROTATE THE ARM 
            if ( (round(diff_angles[0],3) != 0.00 ) or (round(diff_angles[1],2) != 0.00) or (round(diff_angles[2],3) != 0.015) ):
                count = count + 1
                print('Moving...')

                max_time_rot = 0.7
                d_t = max([np.fabs(diff_angles[0])/max_time_rot,np.fabs(diff_angles[1])/max_time_rot,np.fabs(diff_angles[2])/max_time_rot]) #in seconds 

                #x compensation
                if (round(diff_angles[0],3) != 0.00 ):
                    vel_msg.angular.x = (round(diff_angles[0],2))/d_t
                    if (abs(round(vel_msg.angular.x,2)) <= 0.02):
                        vel_msg.angular.x = 0.00
                        feedback.movement = 'Ang Speed too small for X movement!'
                        self.a_server.publish_feedback(feedback)
                        small_x = True
                    else:
                        feedback.movement = 'Moving about X with speed:' + str(vel_msg.angular.x)
                        self.a_server.publish_feedback(feedback)
                        
                else: 
                    vel_msg.angular.x = 0
                    small_x = True
                    feedback.movement = 'No Ang X Movement!'
                    self.a_server.publish_feedback(feedback)

                
                #y compensation
                if (round(diff_angles[1], 2) != 0.00):
                    vel_msg.angular.y = diff_angles[1]/d_t
                    if (abs(round(vel_msg.angular.y,3)) <= 0.005):
                        vel_msg.angular.y = 0.00
                        feedback.movement = 'Ang Speed too small for Y movement!'
                        self.a_server.publish_feedback(feedback)
                        small_y = True
                    else:
                        feedback.movement = 'Moving about Y with speed:' + str(vel_msg.angular.y)
                        self.a_server.publish_feedback(feedback)
                else:
                    vel_msg.angular.y = 0
                    small_y = True
                    feedback.movement = 'No Ang Y Movement!'
                    self.a_server.publish_feedback(feedback)
                    

                #z compensation
                if (round(diff_angles[2], 3) != 0.015):
                    vel_msg.angular.z = (-0.015 + round(diff_angles[2],3))/d_t
                    if (round(vel_msg.angular.z,2) == 0.00):
                        vel_msg.angular.z = 0.00
                        feedback.movement = 'Ang Speed too small for Z movement!'
                        self.a_server.publish_feedback(feedback)
                        small_z = True
                    else:
                        feedback.movement = 'Moving about Z with speed:' + str(vel_msg.angular.z)
                        self.a_server.publish_feedback(feedback)
                else:
                    vel_msg.angular.z = 0
                    small_z = True
                    feedback.movement = 'No Ang Z Movement!'
                    self.a_server.publish_feedback(feedback)
                    

                if (count > 1):
                    #print('Count of this iteration:', count)
                    moved_x = old_angles_x - diff_angles[0]
                    moved_y = old_angles_y - diff_angles[1]
                    moved_z = old_angles_z - diff_angles[2]
                    #print('The angles moved in ', count - 1, ' iteration were:', moved_x, moved_y, moved_z)

                vel_pub.publish(vel_msg)

                old_angles_x = diff_angles[0]
                old_angles_y = diff_angles[1]
                old_angles_z = diff_angles[2]

                if ( (small_x == True) and (small_y == True) and (small_z == True) ):
                    completion_status_ang = True
                
            else: 
                #Stop the movement
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0
                vel_pub.publish(vel_msg)
                completion_status_ang = True
            
            rate.sleep()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)   
        time.sleep(1)  

        feedback.movement = 'Movement Complete!'
        self.a_server.publish_feedback(feedback)
        time.sleep(2)

        #Subaction
        joint = JointState()
        joint.header.stamp = rospy.Time.now()
        joint.name = ["panda_joint1", "panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"] 

        if (goal.type == 'PICK'):
            #"grasp"
            if (not (goal.gripper_content) and goal.obj_reach):
                print("Object has to be GRASPED.")
                subaction_pub.publish("grasp")
            else:
                if (goal.gripper_content):
                    print("Gripper is already holding an object!")
                if (not (goal.obj_reach)):
                    print("Object is unreachable! Please check the reachability of the object.")


        elif (goal.type == 'PLACE'):
            #"release"
            print("Object has to be RELEASED.")
            subaction_pub.publish("release")

        elif (goal.type == 'MOVE'):
            print("Object was MOVED.")

        elif (goal.type == 'SCREW'):
            if (goal.screwable):
                print("Object has to be SCREWED.")
                #subaction_pub.publish("screw")

                #Screw in counterclockwise direction
                if (goal.direction == 'CLOCKWISE'):
                    print("SCREW action on-going...")
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.y = 0.0
                    vel_msg.linear.z = -0.005
                    vel_msg.angular.x = 0.0
                    vel_msg.angular.y = 0.0
                    vel_msg.angular.z = 0.5
                    
                    vel_pub.publish(vel_msg)
                    time.sleep(12.5)

                    print("SCREW action completed. Moving to RESTING position")
                    time.sleep(0.2)
                    #Pause action
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.y = 0.0
                    vel_msg.linear.z = 0.0
                    vel_msg.angular.x = 0.0
                    vel_msg.angular.y = 0.0
                    vel_msg.angular.z = 0.0

                    vel_pub.publish(vel_msg)
                    time.sleep(1)
                    
                else:
                    print("UNSCREW action on-going...")
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.y = 0.0
                    vel_msg.linear.z = 0.005
                    vel_msg.angular.x = 0.0
                    vel_msg.angular.y = 0.0
                    vel_msg.angular.z = 0.5
                    
                    vel_pub.publish(vel_msg)
                    time.sleep(12.5)

                    print("SCREW action completed. Moving to RESTING position")
                    time.sleep(0.2)
                    #Pause action
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.y = 0.0
                    vel_msg.linear.z = 0.0
                    vel_msg.angular.x = 0.0
                    vel_msg.angular.y = 0.0
                    vel_msg.angular.z = 0.0

                    vel_pub.publish(vel_msg)
                    time.sleep(1)

            else:
                print("Object is NOT screwable! Moving back to RESTING position.")      

            #Move to RESTING position
            subaction_pub.publish("rest")
            time.sleep(1)

            
        elif (goal.type == 'REST'):
            #Specify RESTING coordinates as 'goal' in 'arm_client.py'
            print("Moved to the resting position")

    
        #Result = final end_effector coordinates
        if (completion_status_lin and completion_status_ang):
            if success:
                try:
                    ee2b = tfBuffer.lookup_transform('panda_link0', 'sim_end_effector', rospy.Time())

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()

                result.final_coord.pose.position.x = ee2b.transform.translation.x
                result.final_coord.pose.position.y = ee2b.transform.translation.y
                result.final_coord.pose.position.z = ee2b.transform.translation.z
                result.final_coord.pose.orientation.x = ee2b.transform.rotation.x
                result.final_coord.pose.orientation.y = ee2b.transform.rotation.y
                result.final_coord.pose.orientation.z = ee2b.transform.rotation.z
                result.final_coord.pose.orientation.w = ee2b.transform.rotation.w
                self.a_server.set_succeeded(result)
     
        rate.sleep() 

if __name__=="__main__":

    rospy.init_node('action_server')

    vel_pub = rospy.Publisher('/mover/cart_vel', geometry_msgs.msg.Twist, queue_size=1)
    viz_pub = rospy.Publisher("/viz/joint_states", JointState, queue_size = 5)
    subaction_pub = rospy.Publisher("/subaction", String, queue_size = 1)
 
    s = action_server()

    rospy.spin()

