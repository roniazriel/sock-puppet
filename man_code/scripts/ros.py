#!/usr/bin/env python
from logging import warning
from datetime import datetime
import roslaunch
import os
import subprocess
import shlex
from os import listdir, path, mkdir  # , rename
import time
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from datetime import datetime
import csv
import numpy as np
import pandas as pd
import random
from itertools import product
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
import math

class Ros(object):

    """ This class is for using ros commands"""
    def __init__(self):
        """if needed add publish and subcribe"""
        try:
            self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"  
        except ValueError:
            rospy.loginfo('Error occurred at init - Ros object')  # shows warning message
            pass


    def checkroscorerun(self):
        try:
            roscore_pid = rosgraph.Master('/rostopic').getPid()
            return roscore_pid
        except error as e:
            pass

    def start_launch(self,launch_name, pathname, launch_path, args=None):
        """Start launch file"""
        if args is None:
            args = []
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            path = pathname+launch_path+"/launch/"+launch_name+".launch"
            cli_args = [path, args]

            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], args)]
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            # sleep(50)
            return launch
        except ValueError:
            rospy.loginfo('Error occurred at start launch function')
            pass


    def stop_launch(self,launch):
        try:
            launch.shutdown()
            return False
        except ValueError:
            rospy.loginfo('Error occurred at launch_stop function')
            pass

    def ter_command(command):
        """Write Command to the terminal"""
        try:
            command = shlex.split(command)
            ter_command_proc = subprocess.Popen(command, stdout=subprocess.PIPE, preexec_fn=os.setsid)
            return ter_command_proc
        except ValueError:
            rospy.loginfo('Error occurred at ter_command function')  # shows warning message
            pass

    def ros_core_start(self):
        try:
            self.roscore = subprocess.Popen('roscore')
            # rospy.init_node('arl_python', anonymous=True)
            time.sleep(1)  # wait a bit to be sure the roscore is really launched
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_start function')  # shows warning message
            pass

    def ros_core_stop(self,):
        try:
            self.roscore.terminate()
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_stop function')  # shows warning message
            pass

    @staticmethod
    def checkroscorerun():
        try:
            roscore_pid = rosgraph.Master('/rostopic').getPid()
            return roscore_pid
        except error as e:
            pass
    @staticmethod
    def create_folder(name):
        if not os.path.exists(name):
            mkdir(name)
        return name


class MoveGroupPythonInterface(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        try:
          from math import pi, tau, dist, fabs, cos
        except: # For Python 2 compatibility
          from math import pi, fabs, cos, sqrt
          tau = 2.0*pi
          def dist(p, q):
            return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(1)

        # Getting Basic Information
        # self.planning_frame = self.move_group.get_planning_frame()
        # self.move_group.set_planner_id("RRTstarkConfigDefault")
        # # self.move_group.set_num_planning_attempts(10)
        # self.move_group.clear_pose_targets()


        rospy.init_node('spray', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        # self.arm_name = arm_name
        

        current_joints_state = []
        current_joints_names = []
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        # grapes_coords=  [[1.2,2.9,1.5],
        #                  [1.14,2.08,1.57],
        #                  [1.5,1.42,1.8],
        #                  [1.5,0.08,1.2],

        #                  [1.8,-0.15,1.2],
        #                  [1.43,-1.53,1.47],
        #                  [1.18,-1.42,1.6],
        #                  [1.1,-2.15,1.5],
        #                  [1.16,-3.2,1.7],
        #                  [1.8,-4,1.8]]
        # self.grapes_positions = grapes_coords
        # self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"


    def get_current_position(self):
        return self.move_group.get_current_pose().pose.position

    def get_current_orientain(self):
        # a = self.move_group.get_current_pose().pose.orientation  # return orientation in quaternions
        # orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])) - 2 * np.pi) % (2 * np.pi)
        return self.move_group.get_current_rpy()  # orien  # (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])))


    def joint_state_callback(self,data):
        global current_joints_state
        global current_joints_names
        current_joints_state = data.position
        current_joints_names = data.name 

    def go_home(self):
        # home = self.move_group.get_current_joint_values()   
        is_reached=self.move_group.go(self.home, wait=True)
        self.move_group.stop()
        if(is_reached):
            print('Home position reached')
        else:
            print('FAILED to reach home')

    def go_pose(self,goal, joints, links,db,pose_id):
        try:
            spray_offset_x = 0.2
            vine_offset_x = 0.8
            start = time.time()

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 0.707
            pose_goal.orientation.y = 0.707
            # pose_goal.orientation.w = 1
            pose_goal.position.x = goal[0] - spray_offset_x - vine_offset_x
            pose_goal.position.y = goal[1]
            pose_goal.position.z = goal[2]

            self.move_group.set_pose_target(pose_goal)

            # with movement:
            self.plan_result = self.move_group.go(wait=True)
            self.move_group.stop()

            # only plan- without reaching the target
            # self.plan_result = self.move_group.plan()
            # self.plan_result = not(self.plan_result.joint_trajectory.points == [])

            end = time.time()
            print('Time:',end - start)
            Time = end-start
            self.move_group.clear_pose_targets()

            if(self.plan_result):
                print("Success - A plan was created")
                print(current_joints_state)
                print(current_joints_names)
                mu,z,jacobian,cur_pose,mu_roni = self.indices_calc(joints, links)
                time.sleep(0.1) # spraying time
                print('Goal position reached')
            else:
                print("Failed - The goal is out of reach")
                mu,z,jacobian,cur_pose,mu_roni = "","","","",""
                time.sleep(0.1) # pause

            pose_db = self.write_indicators_to_csv(db,pose_id,Time,self.plan_result,mu,z,jacobian,cur_pose,mu_roni) 
            return pose_db
        except:
            print("Ros has crashed in go pose function!!!!!!!!!!!!!!!!!!!!!!!")
            command = ("kill -9 `ps aux | grep ros | grep -v grep | awk '{print $2}'`")
            self.ter_command(command)


    def indices_calc(self, joints, links):
        try:
            # ignoring the final joint which is a roll 
            #cur_pos = self.move_group.get_current_joint_values(

            cur_pos = current_joints_state
            cur_pos = list(cur_pos)
            fake_joint = cur_pos[0]
            cur_pos = cur_pos[1:]
            cur_pos.append(fake_joint)
            print(cur_pos, "cur_pos")

            jacobian = np.delete(self.move_group.get_jacobian_matrix(cur_pos), -1, 1)
            cur_pos = np.asarray(cur_pos)

            # Jacobian singular values (~eighen values)
            j_ev = np.linalg.svd(jacobian, compute_uv=False)

            # Manipulability index
            mu_roni= math.sqrt(np.linalg.det(np.matmul(jacobian,(np.transpose(jacobian)))))
            mu = round(np.product(j_ev), 3)

            # Joint Mid-Range Proximity
            z = self.mid_joint_proximity(cur_pos, joints, links)
            return mu, np.diag(z), jacobian, cur_pos,mu_roni
        except:
            print("indices_calc - except")
            # if there numeric error like one of the values is NaN or Inf or divided by zero
            return -1,np.asarray([-1]*len(joints)), jacobian, cur_pos , -1

    @staticmethod
    def mid_joint_proximity(cur_pos, joints, link_length):
        theta_mean = [0.75]
        to_norm = [1.5]
        for joint in joints:
            if "pris" not in joint:
                theta_mean.append(0)
                to_norm.append(2*np.pi)
            else:
                theta_mean.append(float(link_length[joints.index(joint)])/2)
                to_norm.append(float(link_length[joints.index(joint)]))
        dis = (cur_pos[:-1]-theta_mean)
        nor_dis = np.asarray(np.abs(dis))/np.asarray(to_norm)
        w = np.identity(len(joints)+1)*nor_dis  # weighted diagonal matrix
        print(w,"w")
        print(nor_dis,"nor_dis")
        z = np.around(0.5*np.transpose(nor_dis)*w, 3)
        print(z, "z")
        return z