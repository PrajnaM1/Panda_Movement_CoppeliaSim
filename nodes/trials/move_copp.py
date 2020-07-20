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
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class action_server():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("arm_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()

    def execute_cb(self, goal):

        success = True

        #Status of Movement Completion
        move_status = False
        
        feedback = robot_geoFeedback()
    
        result = robot_geoResult()
        completion_status_lin = False
        completion_status_ang = False

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        pose_msg = geometry_msgs.msg.Pose()
        selection_msg = geometry_msgs.msg.Vector3()
        wrench_msg = geometry_msgs.msg.Wrench()
        
        #current_pose from tf
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print(current_pose)

        #goal_pose from client
        goal_pose = [round(goal.coord.pose.position.x,3), round(goal.coord.pose.position.y,3), round(goal.coord.pose.position.z,3), round(goal.coord.pose.orientation.x,3), round(goal.coord.pose.orientation.y,3), round(goal.coord.pose.orientation.z,3), round(goal.coord.pose.orientation.w,3)]
        
        #ACTION: MOVE TO GOAL/CONTACT 
        #We know where the goal is and where the robot arm is. Interpolate path between these 2 points and move along these points. 
        if (goal.type == 'move to contact' or goal.type == 'wipe'):
            
            x_diff = round((goal_pose[0] - current_pose[0]),3)
            y_diff = round((goal_pose[1] - current_pose[1]),3)
            z_diff = round((goal_pose[2] - current_pose[2]),3)

            x_ang_diff = round((goal_pose[3] - current_pose[3]),3)
            y_ang_diff = round((goal_pose[4] - current_pose[4]),3)
            z_ang_diff = round((goal_pose[5] - current_pose[5]),3)
            w_ang_diff = round((goal_pose[6] - current_pose[6]),3)

            #or (abs(x_ang_diff) > 0.02)  or (abs(y_ang_diff) > 0.02) or (abs(z_ang_diff) > 0.02) or (abs(w_ang_diff) > 0.02) 

            #while ( (abs(x_diff) > 0.015)  or (abs(y_diff) > 0.015) or (abs(z_diff) > 0.015) or (abs(x_ang_diff) > 0.02)  or (abs(y_ang_diff) > 0.02) or (abs(z_ang_diff) > 0.02) or (abs(w_ang_diff) > 0.02)): 
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
                #elif abs(r) >= 0.01 and abs(r) < 0.1:
                #    step_size = 0.005
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

                #print("The two points are:")
                #print(key_rots.as_euler('xyz', degrees=True))
                
                #print("The interpolated angles are:", interp_rots.as_quat())
                quat = interp_rots.as_quat()

                #Publish the point to ee_path_goals
                pose_msg.position.x = round (x[1], 3)
                pose_msg.position.y = round (y[1], 3)
                pose_msg.position.z = round (z[1], 3)
                pose_msg.orientation.x = round(quat[6][0], 3)
                pose_msg.orientation.y = round(quat[6][1], 3)
                pose_msg.orientation.z = round(quat[6][2], 3)
                pose_msg.orientation.w = round(quat[6][3], 3)

                pose_pub.publish(pose_msg)
                print("Moving to point: ", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w )
                time.sleep(5)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                print("Current pose is:", current_pose)

                x_diff = round((goal_pose[0] - current_pose[0]),3)
                y_diff = round((goal_pose[1] - current_pose[1]),3)
                z_diff = round((goal_pose[2] - current_pose[2]),3)

                x_ang_diff = round((goal_pose[3] - current_pose[3]),3)
                y_ang_diff = round((goal_pose[4] - current_pose[4]),3)
                z_ang_diff = round((goal_pose[5] - current_pose[5]),3)
                w_ang_diff = round((goal_pose[6] - current_pose[6]),3)

                
            move_status = True
            print('Movement Complete!')
            time.sleep(1)
            
            if (goal.type == "wipe"):
                #Publish Selection Vector
                selection_msg.x = goal.selection.x 
                selection_msg.y = goal.selection.y 
                selection_msg.z = goal.selection.z 

                selection_pub.publish(selection_msg)

                #Publish Wrench
                wrench_msg.force.x = goal.ee_wrench.force.x
                wrench_msg.force.y = goal.ee_wrench.force.y
                wrench_msg.force.z = goal.ee_wrench.force.z
                wrench_msg.torque.x = goal.ee_wrench.torque.x
                wrench_msg.torque.y = goal.ee_wrench.torque.y
                wrench_msg.torque.z = goal.ee_wrench.torque.z
                wrench_pub.publish(wrench_msg)

                #Publish the movement
                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                time.sleep(0.5)

                pose_msg.position.x = current_pose[0]
                pose_msg.position.y = current_pose[1] + 0.2
                pose_msg.position.z = current_pose[2]
                pose_msg.orientation.x = current_pose[3]
                pose_msg.orientation.y = current_pose[4]
                pose_msg.orientation.z = current_pose[5]
                pose_msg.orientation.w = current_pose[6]
                pose_pub.publish(pose_msg)
                time.sleep(5)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                time.sleep(0.5)

                pose_msg.position.x = current_pose[0]
                pose_msg.position.y = current_pose[1] - 0.2
                pose_msg.position.z = current_pose[2]
                pose_msg.orientation.x = current_pose[3]
                pose_msg.orientation.y = current_pose[4]
                pose_msg.orientation.z = current_pose[5]
                pose_msg.orientation.w = current_pose[6]
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

            feedback.movement = 'Movement and Wiping Complete!'
            self.a_server.publish_feedback(feedback)
            time.sleep(1)

      
            #Result = final end_effector coordinates
            result.final_coord.pose.position.x = trans[0]
            result.final_coord.pose.position.y = trans[1]
            result.final_coord.pose.position.z = trans[2]
            result.final_coord.pose.orientation.x = rot[0]
            result.final_coord.pose.orientation.y = rot[1]
            result.final_coord.pose.orientation.z = rot[2]
            result.final_coord.pose.orientation.w = rot[3]
            self.a_server.set_succeeded(result)
        
            rate.sleep() 

if __name__=="__main__":

    rospy.init_node('action_server')

    pose_pub = rospy.Publisher('/panda/ee_path_goals', geometry_msgs.msg.Pose, queue_size=1)
    selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
    wrench_pub = rospy.Publisher('/panda/ee_wrench_goals', geometry_msgs.msg.Wrench, queue_size=1)
   
    s = action_server()

    rospy.spin()

