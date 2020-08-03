#!/usr/bin/env python
import rospy
import signal
import sys
import math
import tf
import PyKDL
import copy
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import actionlib
from demo.msg import robot_geoAction, robot_geoFeedback, robot_geoResult

import geometry_msgs.msg 
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

class action_server():

    def __init__(self):

        self._tl = tf.TransformListener()
        self._br = tf.TransformBroadcaster()
        rospy.sleep(0.5) 

        #Global Variables Init
        self._success = True
        self._goal_action = ''
        self._goal_pose = []

        self._feedback = robot_geoFeedback()
        self._result = robot_geoResult() 

        self._goal_sel_x = 1.0
        self._goal_sel_y = 1.0
        self._goal_sel_z = 1.0

        self._goal_force_x = 0.0
        self._goal_force_y = 0.0
        self._goal_force_z = 0.0
        self._goal_torque_x = 0.0
        self._goal_torque_y = 0.0
        self._goal_torque_z = 0.0

        self._move_to_goal_status = False
        self._move_to_contact_status = False
        self._action_status = False
    
        self._current_js = []

        #PICK/PLACE EXCLUSIVE
        self._goal_gripper_content = False
        self._goal_obj_reach = True

        #SCREW EXCLUSIVE
        self._goal_depth = 25.4
        self._goal_lead = 1.5
        self._goal_screw_direction = ''
        self._num_rotations = 1
        self._count_rotation = 0
        self._quat = np.zeros((3,3))
        self._screw_count = 0

        #MOVE_TO_CONTACT EXCLUSIVE
        self._count = 0
        self._rf = 0.0 #Radial Force
        self._current_force = geometry_msgs.msg.Vector3
        
        #Publishers
        self._pose_pub = rospy.Publisher('/panda/ee_pose_goals', geometry_msgs.msg.PoseStamped, queue_size=1)

        self._selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
        self._selection_msg = geometry_msgs.msg.Vector3()

        self._wrench_pub = rospy.Publisher('/panda/ee_wrench_goals', geometry_msgs.msg.Wrench, queue_size=1)
        self._wrench_msg = geometry_msgs.msg.Wrench()

        self._subaction_pub = rospy.Publisher("/panda/commands", String, queue_size = 1)
        self._pose_msg = geometry_msgs.msg.PoseStamped()

        #Subscribers
        self._wrench_sub = rospy.Subscriber("/panda/wrench", geometry_msgs.msg.Wrench, self.current_force, queue_size = 1)
        self._current_joint_states_sub = rospy.Subscriber("/panda/joint_states", JointState, self.current_joint_states, queue_size = 1)

        #Action_Server
        self.a_server = actionlib.SimpleActionServer("action_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        
    def execute_cb(self, goal):

        rate = rospy.Rate(100.0) 

        #Init GOAL parameters
        self._goal_action = goal.type
        self._goal_pose = [round(goal.coord.pose.position.x,3), round(goal.coord.pose.position.y,3), round(goal.coord.pose.position.z,3), 
        round(goal.coord.pose.orientation.x,3), round(goal.coord.pose.orientation.y,3), round(goal.coord.pose.orientation.z,3), 
        round(goal.coord.pose.orientation.w,3)]

        self._goal_gripper_content = goal.gripper_content
        self._goal_obj_reach = goal.obj_reach

        self._goal_sel_x = goal.selection.x
        self._goal_sel_y = goal.selection.y
        self._goal_sel_z = goal.selection.z

        self._goal_force_x = goal.ee_wrench.force.x
        self._goal_force_y = goal.ee_wrench.force.y
        self._goal_force_z = goal.ee_wrench.force.z
        self._goal_torque_x = goal.ee_wrench.torque.x
        self._goal_torque_y = goal.ee_wrench.torque.y
        self._goal_torque_z = goal.ee_wrench.torque.z

        self._goal_screw_direction = goal.direction
       
        #90 degree interpolated steps (quat FIXED, will be changed later)
        key_rots = R.from_quat([ [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0]])
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        times = [0.0, 0.5, 1.0]
        interp_rots = slerp(times)
        self._quat = interp_rots.as_quat() 
        print('quat values are: ', self._quat)

        while (not self._action_status and not rospy.is_shutdown()):

            if self.a_server.is_preempt_requested():
                self._success = False
                rospy.loginfo('Preempted')
                self.a_server.set_preempted()
                print("Server Preempted.")

            else:
                if goal.type == 'pick':
                    print("PICK action...")
                    self.action_pick()

                elif goal.type == 'place':
                    print("PLACE action...")
                    self.action_place()

                elif goal.type == 'apply_force':
                    print("APPLY_FORCE action...")
                    
                    #Move to point closer to contact ('goal')
                    if not self._move_to_goal_status:
                        self.primitive_move_to_goal()
                    else:
                        print("Move to GOAL completed!")
                        if not self._move_to_contact_status:
                            self.primitive_move_to_contact()
                        else:
                            print("Move to CONTACT completed! Now, force application..")
                            self.action_apply_force()

                elif goal.type == 'screw':
                    print("SCREW action...")

                    #Move to NAIL OBJ ('goal')
                    if not self._move_to_goal_status:
                        print("Moving towards GOAL...")
                        self.primitive_move_to_goal()
                    
                    elif self._count_rotation > 0 and self._screw_count == 0:
                        print('Rotating next round...')
                        act = self.action_screw()

                    elif self._screw_count >= 1 and self._screw_count <= 3:
                        print("Rotating (90 deg rotation)...")
                        act = self.action_screw()
                    
                    else:
                        #Grasp the nail
                        self._subaction = "grasp"
                        self._subaction_pub.publish(self._subaction)
                        print("Grasping...")
                        if (abs(round(self._current_js[-2], 3)) == 0.084 and abs(round(self._current_js[-1], 3)) == 0.084): #The values are checked with panda.ttt
                            act = self.action_screw()
                else:
                    print("Invalid Action!")

            rate.sleep()

        #Publish the RESULT
        if self._success:

            (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
            current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
            print("RESULT is: ", current_pose)

            #Result = final end_effector coordinates
            self._result.final_coord.pose.position.x = trans[0]
            self._result.final_coord.pose.position.y = trans[1]
            self._result.final_coord.pose.position.z = trans[2]
            self._result.final_coord.pose.orientation.x = rot[0]
            self._result.final_coord.pose.orientation.y = rot[1]
            self._result.final_coord.pose.orientation.z = rot[2]
            self._result.final_coord.pose.orientation.w = rot[3]

            self.a_server.set_succeeded(self._result)

####################################################################################################
            
    def current_force(self, msg):

        self._current_force = msg.force
    
#####################################################################################################
    
    def current_joint_states(self, msg):

        self._current_js = msg.position

#####################################################################################################

    def action_pick(self):

        self.primitive_move_to_goal()

        if self._move_to_goal_status:
            if (not self._goal_gripper_content and self._goal_obj_reach):
                print("Object will be PICKED.")
                self._subaction_pub.publish("grasp")
                self._action_status = True

            else:
                self._action_status = False
                if (self._goal_gripper_content):
                    print("Gripper already has an object!")
                if (not (self._goal_obj_reach)):
                    print("Object is unreachable! Please check the reachability of the object.")

#########################################################################################################

    def action_place(self):

        self.primitive_move_to_goal()

        if self._move_to_goal_status:
            if (self._goal_gripper_content and self._goal_obj_reach):
                print("Object will be PLACED.")
                self._subaction_pub.publish("release")
                self._action_status = True

            else:
                self._action_status = False
                if (not self._goal_gripper_content):
                    print("Gripper has NO object!")
                if (not (self._goal_obj_reach)):
                    print("Object is unreachable! Please check the reachability of the object.")

#########################################################################################################

    def action_apply_force(self):

        print("Applying Force..")
            
        #Publish the Selection Vector
        self._selection_msg.x = self._goal_sel_x 
        self._selection_msg.y = self._goal_sel_y 
        self._selection_msg.z = self._goal_sel_z
        self._selection_pub.publish(self._selection_msg)

        #Publish Wrench
        self._wrench_msg.force.x = 0.0
        self._wrench_msg.force.y = 0.0
        self._wrench_msg.force.z = self._goal_force_z
        self._wrench_msg.torque.x = self._goal_torque_x
        self._wrench_msg.torque.y = self._goal_torque_y
        self._wrench_msg.torque.z = self._goal_torque_z
        self._wrench_pub.publish(self._wrench_msg)

        self._action_status = True

#########################################################################################################

    def action_screw(self):
        
        #Current Pose
        (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose_prev = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), 
        round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print('current_pose_prev: ', current_pose_prev)

        #Screw/UNscrew action
        if self._goal_screw_direction == 'clockwise' and self._screw_count < 3:

            print("Screw count is: ", self._screw_count)

            self._selection_msg.x = 1.0
            self._selection_msg.y = 1.0 
            self._selection_msg.z = 0.0
            self._selection_pub.publish(self._selection_msg)

            self._wrench_msg.force.x = 0.0
            self._wrench_msg.force.y = 0.0
            self._wrench_msg.force.z = -5.0
            self._wrench_msg.torque.x = 0.0
            self._wrench_msg.torque.y = 0.0
            self._wrench_msg.torque.z = 0.0
            self._wrench_pub.publish(self._wrench_msg)

            self._pose_msg.header.stamp = rospy.Time.now()
            self._pose_msg.pose.position.x = current_pose_prev[0]
            self._pose_msg.pose.position.y = current_pose_prev[1] 
            self._pose_msg.pose.position.z = current_pose_prev[2] 
            self._pose_msg.pose.orientation.x = round(self._quat[self._screw_count][0], 3)
            self._pose_msg.pose.orientation.y = round(self._quat[self._screw_count][1], 3)
            self._pose_msg.pose.orientation.z = round(self._quat[self._screw_count][2], 3)
            self._pose_msg.pose.orientation.w = round(self._quat[self._screw_count][3], 3)
            self._pose_pub.publish(self._pose_msg)

            self._screw_count += 1
            '''
            self._selection_msg.x = 1.0
            self._selection_msg.y = 1.0 
            self._selection_msg.z = 1.0
            self._selection_pub.publish(self._selection_msg)

            self._wrench_msg.force.x = 0.0
            self._wrench_msg.force.y = 0.0
            self._wrench_msg.force.z = 0.0
            self._wrench_msg.torque.x = 0.0
            self._wrench_msg.torque.y = 0.0
            self._wrench_msg.torque.z = 0.0
            self._wrench_pub.publish(self._wrench_msg)
            '''
            
            return True

            #Exit

        elif self._goal_screw_direction == 'clockwise' and self._screw_count == 3: 

            self._selection_msg.x = 1.0
            self._selection_msg.y = 1.0 
            self._selection_msg.z = 1.0
            self._selection_pub.publish(self._selection_msg)

            self._wrench_msg.force.x = 0.0
            self._wrench_msg.force.y = 0.0
            self._wrench_msg.force.z = 0.0
            self._wrench_msg.torque.x = 0.0
            self._wrench_msg.torque.y = 0.0
            self._wrench_msg.torque.z = 0.0
            self._wrench_pub.publish(self._wrench_msg)

            self._subaction = "release"
            self._subaction_pub.publish(self._subaction)
                
            if (abs(round(self._current_js[-2], 1)) == 0.0 and abs(round(self._current_js[-1], 1)) == 0.0): #The values are checked with panda.ttt

                print ("Released obj.")
                self._count_rotation = self._count_rotation + 1
                print("90 deg Count rotation: ", self._count_rotation)

                #Get current pose of NAIL OBJ
                (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
                current_nail_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), 
                round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                
                if self._count_rotation == 4:
                    print("360 rotation complete!")
                    self._action_status = True
                    self._screw_count = 0
                    #Move UP TO RESTING pose
                    self._pose_msg.header.stamp = rospy.Time.now()
                    self._pose_msg.pose.position.x = 0.5
                    self._pose_msg.pose.position.y = 0.0
                    self._pose_msg.pose.position.z = 0.5
                    self._pose_msg.pose.orientation.x = 0
                    self._pose_msg.pose.orientation.y = 0
                    self._pose_msg.pose.orientation.z = 0
                    self._pose_msg.pose.orientation.w = 1
                    self._pose_pub.publish(self._pose_msg)
                    return True
                else:
                    print("360 rotation NOT complete! Moving back to NAIL OBJ")
                    self._pose_msg.header.stamp = rospy.Time.now()
                    self._pose_msg.pose.position.x = current_nail_pose[0]
                    self._pose_msg.pose.position.y = current_nail_pose[1]
                    self._pose_msg.pose.position.z = current_nail_pose[2]
                    self._pose_msg.pose.orientation.x = 0
                    self._pose_msg.pose.orientation.y = 0
                    self._pose_msg.pose.orientation.z = 0
                    self._pose_msg.pose.orientation.w = 1
                    self._pose_pub.publish(self._pose_msg)

                    self._action_status = False
                    self._screw_count = 0
                    return False
        
#########################################################################################################

    def primitive_move_to_goal(self):

            self._feedback.movement = 'Goal Pose is: ' + str(self._goal_pose)
            self.a_server.publish_feedback(self._feedback)

            #Current Pose
            (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
            current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), 
            round(rot[1],3), round(rot[2],3), round(rot[3],3)]

            #Pose Differences
            x_diff = round((self._goal_pose[0] - current_pose[0]),3)
            y_diff = round((self._goal_pose[1] - current_pose[1]),3)
            z_diff = round((self._goal_pose[2] - current_pose[2]),3)

            x_ang_diff = round((self._goal_pose[3] - current_pose[3]),3)
            y_ang_diff = round((self._goal_pose[4] - current_pose[4]),3)
            z_ang_diff = round((self._goal_pose[5] - current_pose[5]),3)
            w_ang_diff = round((self._goal_pose[6] - current_pose[6]),3)

            if ( (abs(x_diff) > 0.015)  or (abs(y_diff) > 0.015) or (abs(z_diff) > 0.015) ):

                #LINEAR Movement
                r = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
                
                if abs(r) >= 0.1:
                    step_size = 0.05
                else:
                    step_size = 0.01

                num_pts = abs(int((r // step_size) + 1))

                if (abs(x_diff) > 0.015):
                    x_step = x_diff / num_pts
                    x = np.arange(current_pose[0], self._goal_pose[0], x_step)
                else: 
                    print("x_diff nearly 0")
                    x = np.full(num_pts, current_pose[0])

                if (abs(y_diff) > 0.015):
                    y_step = y_diff / num_pts
                    y = np.arange(current_pose[1], self._goal_pose[1], y_step)
                else: 
                    print("y_diff nearly 0")
                    y = np.full(num_pts, current_pose[1])

                if (abs(z_diff) > 0.015):
                    z_step = z_diff / num_pts
                    z = np.arange(current_pose[2], self._goal_pose[2], z_step)
                else: 
                    print("z_diff nearly 0")
                    z = np.full(num_pts, current_pose[2])

                if ( (abs(x_ang_diff) > 0.015)  or (abs(y_ang_diff) > 0.015) or (abs(z_ang_diff) > 0.015) ):

                    #ANGULAR Movement
                    key_rots = R.from_quat([ [current_pose[3], current_pose[4], current_pose[5], 
                    current_pose[6]], [self._goal_pose[3], self._goal_pose[4], self._goal_pose[5],
                    self._goal_pose[6]]])

                    key_times = [0, 1]

                    slerp = Slerp(key_times, key_rots)

                    times = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
                    interp_rots = slerp(times)

                    quat = interp_rots.as_quat()

                    self._pose_msg.pose.orientation.x = round(quat[6][0], 3)
                    self._pose_msg.pose.orientation.y = round(quat[6][1], 3)
                    self._pose_msg.pose.orientation.z = round(quat[6][2], 3)
                    self._pose_msg.pose.orientation.w = round(quat[6][3], 3)

                else:
                    self._pose_msg.pose.orientation.x = current_pose[3]
                    self._pose_msg.pose.orientation.y = current_pose[4]
                    self._pose_msg.pose.orientation.z = current_pose[5]
                    self._pose_msg.pose.orientation.w = current_pose[6]
                    
                #Publish the point to ee_pose_goals
                self._pose_msg.header.stamp = rospy.Time.now()
                self._pose_msg.pose.position.x = round (x[1], 3)
                self._pose_msg.pose.position.y = round (y[1], 3)
                self._pose_msg.pose.position.z = round (z[1], 3)
                
                self._pose_pub.publish(self._pose_msg)
                print("Moving to point: ", self._pose_msg.pose.position.x, self._pose_msg.pose.position.y, 
                self._pose_msg.pose.position.z, self._pose_msg.pose.orientation.x, self._pose_msg.pose.orientation.y, 
                self._pose_msg.pose.orientation.z, self._pose_msg.pose.orientation.w )

                (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3),
                round(rot[2],3), round(rot[3],3)]

                print("Current pose is:", current_pose)
                self._feedback.movement = 'Current Pose is: ' + str(current_pose)
                self.a_server.publish_feedback(self._feedback)

            else:
                self._move_to_goal_status = True
                

#########################################################################################################

    def primitive_move_to_contact(self):

        #Current Pose
        (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), 
        round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        
        forces = np.sqrt(self._current_force.x**2 + self._current_force.y**2 + self._current_force.z**2)
        self._rf = forces
        print('RADIAL Force is: ', self._rf)
        
        if (abs(round(self._rf,1)) < 18.6): #move

            self._feedback.movement = 'Moving further towards contact..'
            self.a_server.publish_feedback(self._feedback)
        
            self._move_to_contact_status = False

            self._rf = 0 
            
            #Publish the point to ee_pose_goals
            self._pose_msg.header.stamp = rospy.Time.now()
            self._pose_msg.pose.position.z = current_pose[2]
            self._pose_msg.pose.orientation.x = current_pose[3]
            self._pose_msg.pose.orientation.y = current_pose[4]
            self._pose_msg.pose.orientation.z = current_pose[5] - 0.05
            self._pose_msg.pose.orientation.w = current_pose[6]
            self._pose_pub.publish(self._pose_msg)
            print("Moving to point: ", self._pose_msg.pose.position.x, self._pose_msg.pose.position.y, 
            self._pose_msg.pose.position.z, self._pose_msg.pose.orientation.x, self._pose_msg.pose.orientation.y, 
            self._pose_msg.pose.orientation.z, self._pose_msg.pose.orientation.w )

            (trans,rot) = self._tl.lookupTransform('panda_link0', 'end_effector', rospy.Time())
            current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
            
            self._feedback.movement = 'Current Pose is: ' + str(current_pose)
            self.a_server.publish_feedback(self._feedback)
        
        else:
            print('Move to contact DONE!')
            self._move_to_contact_status = True
            

#########################################################################################################

if __name__ == "__main__":

    rospy.init_node("action_server_node")

    a = action_server()

    rospy.spin()








