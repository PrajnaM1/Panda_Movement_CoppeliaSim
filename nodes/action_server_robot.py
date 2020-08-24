#!/usr/bin/env python
import rospy
import signal
import sys
import math
import tf2_ros
import tf2_geometry_msgs
import PyKDL
import copy
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import actionlib
from demo.msg import robot_geoAction, robot_geoFeedback, robot_geoResult
from panda_ros_msgs.msg import HybridPose, JointPose

import geometry_msgs.msg 
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

class action_server():

    def __init__(self):

        self._tfBuffer = tf2_ros.Buffer()
        self._tl = tf2_ros.TransformListener(self._tfBuffer)
        rospy.sleep(0.5) 

        self._starting_time = None 
        self._goal_time = None

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

        self._current_js = []

        #PICK/PLACE EXCLUSIVE
        self._goal_gripper_content = False
        self._goal_obj_reach = True

        #SCREW EXCLUSIVE
        self._goal_depth = 25.4
        self._goal_lead = 1.5
        self._goal_screw_direction = ''
        self._num_rotations = 0
        self._count_rotation = 0
        self._quat = np.zeros((3,3))

        #MOVE_TO_CONTACT EXCLUSIVE
        self._count = 0
        self._rf = 0.0 #Radial Force
        self._current_force = geometry_msgs.msg.Vector3
        
        #Publishers
        self._selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
        self._selection_msg = geometry_msgs.msg.Vector3()

        self._hybrid_pose_pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=1)
        self._hybrid_pose_msg = HybridPose()

        self._subaction_pub = rospy.Publisher("/panda/commands", String, queue_size = 1)
        self._pose_msg = geometry_msgs.msg.PoseStamped()

        #Subscribers
        self._wrench_sub = rospy.Subscriber("/panda/control_wrench", geometry_msgs.msg.Wrench, self.current_force)
        self._current_joint_states_sub = rospy.Subscriber("/panda/joint_states", JointState, self.current_joint_states, queue_size = 1)

        #Action_Server
        self.a_server = actionlib.SimpleActionServer("action_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        
    def execute_cb(self, goal):

        rate = rospy.Rate(100.0) 
        self._starting_time = rospy.Time.now()

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
       
        self._grasp_count = 0
        self._grasp_status = False
        self._rel_count = 0
        self._rel_status = False

        self._rotate_screw_status = False
        self._screw_count = 0

        self._move_to_goal_status = False
        
        self._move_to_contact_status = False

        self._action_status = False

        while (not self._action_status and not rospy.is_shutdown()):

            if self.a_server.is_preempt_requested():
                self._success = False
                rospy.loginfo('Preempted')
                self.a_server.set_preempted()
                print("Server Preempted.")

            elif self._goal_action == 'move_to_goal':

                if not self._move_to_goal_status:
                    self.primitive_move_to_goal()
                else:
                    print("move_to_goal complete!")
                    self._action_status = True

            elif self._goal_action == 'move_to_contact':
                
                if not self._move_to_contact_status:
                    self.primitive_move_to_contact()
                else:
                    print("move_to_contact complete!")
                    self._action_status = True

            elif self._goal_action == 'grasp':
                
                if self._grasp_count == 0:
                    print("Grasping..")
                    self.grasp()
                    print("Grasping complete!")
                    self._grasp_status = True
                    self._action_status = True

            elif self._goal_action == 'release':
                
                if self._rel_count == 0:
                    print("Releasing..")
                    self.release()
                    print("Releasing complete!")
                    self._grasp_status = True
                    self._action_status = True

            elif self._goal_action == 'apply_force':
                
                print("Applying Force...")
                self.action_apply_force()

            elif self._goal_action == 'rotate_screw':
                
                print("Rotating screw...")
                self.primitive_rotate_screw()
                
            elif self._goal_action == 'stop_force':
                
                self.primitive_stop_force()
                self._action_status = True
            
            elif self._goal_action == 'rotate_back':
                
                print("Rotating BACK...")
                self.primitive_rotate_back()

            else:
                print("Invalid goal.type!")

            rate.sleep()

        #Publish the RESULT
        if self._success:

            t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
            trans = t.transform.translation
            rot = t.transform.rotation
            current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
            round(rot.y,3), round(rot.z,3), round(rot.w,3)]
            print("RESULT is: ", current_pose)

            #Result = final end_effector coordinates
            self._result.final_coord.pose.position.x = trans.x
            self._result.final_coord.pose.position.y = trans.y
            self._result.final_coord.pose.position.z = trans.z
            self._result.final_coord.pose.orientation.x = rot.x
            self._result.final_coord.pose.orientation.y = rot.y
            self._result.final_coord.pose.orientation.z = rot.z
            self._result.final_coord.pose.orientation.w = rot.w

            self.a_server.set_succeeded(self._result)

####################################################################################################
            
    def current_force(self, msg):

        self._current_force = msg.force
    
#####################################################################################################
    
    def current_joint_states(self, msg):

        self._current_js = msg.position

#####################################################################################################

    def grasp(self):

        self._subaction = "grasp"
        self._subaction_pub.publish(self._subaction)
        self._grasp_count = 1

#####################################################################################################

    def release(self):

        self._subaction = "release"
        self._subaction_pub.publish(self._subaction)
        self._grasp_count = 1

#####################################################################################################

    def primitive_rotate_screw(self):
        
        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
        round(rot.y,3), round(rot.z,3), round(rot.w,3)]
        
        #Record initial orientation for rotate back function
        if self._screw_count == 0:
           self._screw_current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
           round(rot.y,3), round(rot.z,3), round(rot.w,3)]
           self._screw_count += 1
           print("Init Pose for Rotate back: ", self._screw_current_pose)
                
        angle = np.pi / 4 #IMPORTANT: Set opposite angle for 'ROTATE BACK' 
        screw_q = PyKDL.Rotation.RPY(0.0, 0.0, angle).GetQuaternion()
        print("The final angle quaternions for the rotation are: ", screw_q)

        #ee pose
        p = geometry_msgs.msg.PoseStamped()
        p.header.stamp = rospy.Time(0)
        p.header.frame_id = "end_effector"
        p.pose.position.x = 0.0 #Not Used
        p.pose.position.y = 0.0 #Not Used
        p.pose.position.z = 0.0 #Not Used
        p.pose.orientation.x = screw_q[0]
        p.pose.orientation.y = screw_q[1]
        p.pose.orientation.z = screw_q[2]
        p.pose.orientation.w = screw_q[3]

        #Convert to panda_link0 frame
        p = self._tfBuffer.transform(p,"panda_link0")
        p = [round(p.pose.position.x,3), round(p.pose.position.y,3), round(p.pose.position.z,3), round(p.pose.orientation.x,3), 
        round(p.pose.orientation.y,3), round(p.pose.orientation.z,3), round(p.pose.orientation.w,3)]
        
        # Works for 0,0,0,1
        # if  self._goal_screw_direction == 'clockwise':
            # self._goal_pose = [current_pose[0], current_pose[1], current_pose[2] - 0.001, p[3], p[4], p[5], p[6]]
        # else:
            # self._goal_pose = [current_pose[0], current_pose[1], current_pose[2] + 0.001, p[3], p[4], p[5], p[6]]
            
        self._goal_pose = [current_pose[0], current_pose[1], current_pose[2], p[3], p[4], p[5], p[6]]
                    
        print("Currently at: ", current_pose, "New pose at: ", self._goal_pose)

        #Get goal_time
        diff_linear = np.array([self._goal_pose[0] - trans.x, self._goal_pose[1] - trans.y, self._goal_pose[2] - trans.z])
        
        qp = PyKDL.Rotation.Quaternion(rot.x,rot.y,rot.z,rot.w)
        current_angle = qp.GetRPY()
        qg = PyKDL.Rotation.Quaternion(self._goal_pose[3],self._goal_pose[4],self._goal_pose[5],self._goal_pose[6])
        q = qg*qp.Inverse()
        difference_angle = q.GetRPY()
        diff_angular = np.array(difference_angle)
        final_ang = current_angle[2] + difference_angle[2]
        if final_ang > np.pi:
            diff_angular[2] -= 2 * np.pi
        if final_ang < -np.pi:
            diff_angular[2] += 2 * np.pi
            
        max_vel_lin = 0.14
        max_vel_ang = 0.70
        time_lin = np.sqrt(np.sum(diff_linear**2)) / max_vel_lin
        time_ang = max(np.fabs(diff_angular))/max_vel_ang
        f_time = max(time_lin,time_ang)
        self._goal_time = self._starting_time + rospy.Duration(f_time)
                
        if not self._move_to_goal_status:
            print("Moving...")
            self.primitive_move_to_goal()
        else:
            print("Rotation Complete!")
            self._action_status = True
           
#####################################################################################################

    def primitive_rotate_back(self):
        
        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
        round(rot.y,3), round(rot.z,3), round(rot.w,3)]

        angle = -np.pi / 4
        screw_q = PyKDL.Rotation.RPY(0.0, 0.0, angle).GetQuaternion()
        print("The final angle quaternions for the rotation are: ", screw_q)

        #ee pose
        p = geometry_msgs.msg.PoseStamped()
        p.header.stamp = rospy.Time(0)
        p.header.frame_id = "end_effector"
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = 0.0
        p.pose.orientation.x = screw_q[0]
        p.pose.orientation.y = screw_q[1]
        p.pose.orientation.z = screw_q[2]
        p.pose.orientation.w = screw_q[3]

        #Convert to panda_link0 frame
        p = self._tfBuffer.transform(p,"panda_link0")
        p = [round(p.pose.position.x,3), round(p.pose.position.y,3), round(p.pose.position.z,3), round(p.pose.orientation.x,3), 
        round(p.pose.orientation.y,3), round(p.pose.orientation.z,3), round(p.pose.orientation.w,3)]

        self._goal_pose = [current_pose[0], current_pose[1], current_pose[2], p[3], p[4], p[5], p[6]]
        print("Currently at: ", current_pose, "New pose at: ", self._goal_pose)

        #Get goal_time
        diff_linear = np.array([self._goal_pose[0] - trans.x, self._goal_pose[1] - trans.y, self._goal_pose[2] - trans.z])
        qp = PyKDL.Rotation.Quaternion(rot.x,rot.y,rot.z,rot.w)
        current_angle = qp.GetRPY()
        qg = PyKDL.Rotation.Quaternion(self._goal_pose[3],self._goal_pose[4],self._goal_pose[5],self._goal_pose[6])
        q = qg*qp.Inverse()
        difference_angle = q.GetRPY()
        diff_angular = np.array(difference_angle)
        final_ang = current_angle[2] + difference_angle[2]
        if final_ang > np.pi:
            diff_angular[2] -= 2 * np.pi
        if final_ang < -np.pi:
            diff_angular[2] += 2 * np.pi
        max_vel_lin = 0.14
        max_vel_ang = 0.70
        time_lin = np.sqrt(np.sum(diff_linear**2)) / max_vel_lin
        time_ang = max(np.fabs(diff_angular))/max_vel_ang
        f_time = max(time_lin,time_ang)
        self._goal_time = self._starting_time + rospy.Duration(f_time)
                
        if not self._move_to_goal_status:
            print("Moving...")
            self.primitive_move_to_goal()
        else:
            print("Rotate Back Complete!")
            self._action_status = True
                    
#####################################################################################################

    def primitive_stop_force(self):

        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
        round(rot.y,3), round(rot.z,3), round(rot.w,3)]

        self._hybrid_pose_msg.sel_vector = [1, 1, 1, 1, 1, 1]

        self._hybrid_pose_msg.constraint_frame.w = 1

        self._hybrid_pose_msg.wrench.force.x = 0.0
        self._hybrid_pose_msg.wrench.force.y = 0.0
        self._hybrid_pose_msg.wrench.force.z = 0.0
        self._hybrid_pose_msg.wrench.torque.x = 0.0
        self._hybrid_pose_msg.wrench.torque.y = 0.0
        self._hybrid_pose_msg.wrench.torque.z = 0.0
        
        self._hybrid_pose_msg.pose.position.x = current_pose[0]
        self._hybrid_pose_msg.pose.position.y = current_pose[1] 
        self._hybrid_pose_msg.pose.position.z = current_pose[2] 
        self._hybrid_pose_msg.pose.orientation.x = current_pose[3] 
        self._hybrid_pose_msg.pose.orientation.y = current_pose[4] 
        self._hybrid_pose_msg.pose.orientation.z = current_pose[5] 
        self._hybrid_pose_msg.pose.orientation.w = current_pose[6] 
        
        self._hybrid_pose_pub.publish(self._hybrid_pose_msg)

#####################################################################################################

    def primitive_move_to_goal(self):

        self._feedback.movement = 'Goal Pose is: ' + str(self._goal_pose)
        self.a_server.publish_feedback(self._feedback)
        #print("Goal pose in function is: ", self._goal_pose)

        #Current Pose
        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
        round(rot.y,3), round(rot.z,3), round(rot.w,3)]
            
        #Get goal_time   
        diff_linear = np.array([self._goal_pose[0] - trans.x, self._goal_pose[1] - trans.y, self._goal_pose[2] - trans.z])
                
        qp = PyKDL.Rotation.Quaternion(rot.x,rot.y,rot.z,rot.w)
        current_angle = qp.GetRPY()
        qg = PyKDL.Rotation.Quaternion(self._goal_pose[3],self._goal_pose[4],self._goal_pose[5],self._goal_pose[6])
        q = qg*qp.Inverse()
        difference_angle = q.GetRPY()
        diff_angular = np.array(difference_angle)
        final_ang = current_angle[2] + difference_angle[2]
        if final_ang > np.pi:
            diff_angular[2] -= 2 * np.pi
        if final_ang < -np.pi:
            diff_angular[2] += 2 * np.pi
                    
        max_vel_lin = 0.14
        max_vel_ang = 0.70
        time_lin = np.sqrt(np.sum(diff_linear**2)) / max_vel_lin
        time_ang = max(np.fabs(diff_angular))/max_vel_ang
        f_time = max(time_lin,time_ang)
        self._goal_time = self._starting_time + rospy.Duration(f_time)
    
        linear_threshold = 0.001
        angular_threshold = 0.01

        if np.all(np.abs(diff_linear)<linear_threshold) and np.all(np.abs(diff_angular)<angular_threshold):
            self._move_to_goal_status = True
          
        else:
            self._hybrid_pose_msg.sel_vector = [1, 1, 1, 1, 1, 1]

            self._hybrid_pose_msg.constraint_frame.w = 1

            #Step Size Determination
            real_time_elapsed = (rospy.Time.now() - self._starting_time).to_sec()
            action_duration = (self._goal_starting_time - self._starting_time).to_sec()
            steps = min(real_time_elapsed/action_duration, 1)
            steps = min(steps, 1)

            if steps == 1:
               self._move_to_goal_status = True 

            else:
                p = geometry_msgs.msg.PoseStamped()
                p.header = t.header

                p.pose.position.x = trans.x + (self._goal_pose[0] - trans.x)*steps
                p.pose.position.y = trans.y + (self._goal_pose[1] - trans.y)*steps
                p.pose.position.z = trans.z + (self._goal_pose[2] - trans.z)*steps

                key_rots = R.from_quat([ [rot.x, rot.y, rot.z,rot.w], 
                [self._goal_pose[3], self._goal_pose[4], self._goal_pose[5], self._goal_pose[6]]])
                key_times = [0, 1]
                slerp = Slerp(key_times, key_rots)
                #times = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
                quat = slerp([steps]).as_quat()[0]
                p.pose.orientation.x = quat[0]
                p.pose.orientation.y = quat[1]
                p.pose.orientation.z = quat[2]
                p.pose.orientation.w = quat[3]

                self._hybrid_pose_msg.pose.position.x = p.pose.position.x
                self._hybrid_pose_msg.pose.position.y = p.pose.position.y
                self._hybrid_pose_msg.pose.position.z = p.pose.position.z
                self._hybrid_pose_msg.pose.orientation.x = p.pose.orientation.x
                self._hybrid_pose_msg.pose.orientation.y = p.pose.orientation.y
                self._hybrid_pose_msg.pose.orientation.z = p.pose.orientation.z
                self._hybrid_pose_msg.pose.orientation.w = p.pose.orientation.w
                    
                self._hybrid_pose_pub.publish(self._hybrid_pose_msg)

                #print("Moving to point: ", self._hybrid_pose_msg.pose.position.x, self._hybrid_pose_msg.pose.position.y, 
                #self._hybrid_pose_msg.pose.position.z, self._hybrid_pose_msg.pose.orientation.x, self._hybrid_pose_msg.pose.orientation.y, 
                #elf._hybrid_pose_msg.pose.orientation.z, self._hybrid_pose_msg.pose.orientation.w )

                t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time())
                trans = t.transform.translation
                rot = t.transform.rotation
                current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), round(rot.y,3),
                round(rot.z,3), round(rot.w,3)]

                print("Current pose is:", current_pose)
                self._feedback.movement = 'Current Pose is: ' + str(current_pose)
                self.a_server.publish_feedback(self._feedback)

                self._move_to_goal_status = False              

#########################################################################################################

    def primitive_move_to_contact(self):

        #Current Pose
        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), round(rot.y,3),
        round(rot.z,3), round(rot.w,3)]
        
        forces = np.sqrt(self._current_force.x**2 + self._current_force.y**2 + self._current_force.z**2)
        self._rf = forces
        print('RADIAL Force is: ', self._rf)

        self._hybrid_pose_msg.sel_vector = [1, 1, 1, 1, 1, 1]
        self._hybrid_pose_msg.constraint_frame.w = 1
        
        if (self._rf < 30): #move

            self._feedback.movement = 'Moving further towards contact..'
            self.a_server.publish_feedback(self._feedback)
        
            self._move_to_contact_status = False

            self._rf = 0 
            
            #Publish the point to hybrid_pose
            self._hybrid_pose_msg.pose.position.x = current_pose[0]
            self._hybrid_pose_msg.pose.position.y = current_pose[1]
            self._hybrid_pose_msg.pose.position.z = current_pose[2] - 0.01
            self._hybrid_pose_msg.pose.orientation.x = current_pose[3]
            self._hybrid_pose_msg.pose.orientation.y = current_pose[4]
            self._hybrid_pose_msg.pose.orientation.z = current_pose[5] 
            self._hybrid_pose_msg.pose.orientation.w = current_pose[6]
            self._hybrid_pose_pub.publish(self._hybrid_pose_msg)

            print("Moving to point: ", self._hybrid_pose_msg.pose.position.x, self._hybrid_pose_msg.pose.position.y, 
            self._hybrid_pose_msg.pose.position.z, self._hybrid_pose_msg.pose.orientation.x, self._hybrid_pose_msg.pose.orientation.y, 
            self._hybrid_pose_msg.pose.orientation.z, self._hybrid_pose_msg.pose.orientation.w )

            t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
            trans = t.transform.translation
            rot = t.transform.rotation
            current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), round(rot.y,3),
            round(rot.z,3), round(rot.w,3)]
            
            self._feedback.movement = 'Current Pose is: ' + str(current_pose)
            self.a_server.publish_feedback(self._feedback)
        
        else:
            print('Move to contact DONE!')
            self._move_to_contact_status = True
            

#########################################################################################################

    def action_apply_force(self):

        print("Applying Force..")

        #Current Pose
        t = self._tfBuffer.lookup_transform('panda_link0', 'end_effector', rospy.Time(0))
        trans = t.transform.translation
        rot = t.transform.rotation
        current_pose = [round(trans.x,3), round(trans.y,3), round(trans.z,3), round(rot.x,3), 
        round(rot.y,3), round(rot.z,3), round(rot.w,3)]
            
        #Publish the Selection Vector
        self._hybrid_pose_msg.sel_vector = [1, 1, 0, 1, 1, 1]
        self._hybrid_pose_msg.constraint_frame.w = 1

        #Publish Wrench
        self._hybrid_pose_msg.wrench.force.x = 0.0
        self._hybrid_pose_msg.wrench.force.y = 0.0
        self._hybrid_pose_msg.wrench.force.z = -10.0
        self._hybrid_pose_msg.wrench.torque.x = 0.0
        self._hybrid_pose_msg.wrench.torque.y = 0.0
        self._hybrid_pose_msg.wrench.torque.z = 0.0
        
        self._hybrid_pose_msg.pose.position.x = current_pose[0]
        self._hybrid_pose_msg.pose.position.y = current_pose[1] 
        self._hybrid_pose_msg.pose.position.z = current_pose[2] 
        self._hybrid_pose_msg.pose.orientation.x = current_pose[3] 
        self._hybrid_pose_msg.pose.orientation.y = current_pose[4] 
        self._hybrid_pose_msg.pose.orientation.z = current_pose[5] 
        self._hybrid_pose_msg.pose.orientation.w = current_pose[6] 
        
        self._hybrid_pose_pub.publish(self._hybrid_pose_msg)

        self._action_status = True

#########################################################################################################

if __name__ == "__main__":

    rospy.init_node("action_server_node")

    a = action_server()

    rospy.spin()








