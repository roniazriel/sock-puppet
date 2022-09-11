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
import rosgraph
import math
import socket
from time import sleep

class Ros(object):

    """ This class is for using ros commands"""
    def __init__(self):
        """if needed add publish and subcribe"""
        try:
            self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"  
        except ValueError:
            rospy.loginfo('Error occurred at init - Ros object')  # shows warning message
            pass

    @staticmethod
    def checkroscorerun():
        try:
            roscore_pid = rosgraph.Master('/rostopic').getPid()
            return roscore_pid
        except socket.error:
            print("Unable to communicate with master!")

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

    def ter_command(self,command):
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

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name )

        self.move_group.set_goal_orientation_tolerance(0.01)
        self.move_group.set_goal_position_tolerance(0.05)
        # Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        # Getting Basic Information
        self.planning_frame = self.move_group.get_planning_frame()
        self.move_group.set_planner_id("RRTstarkConfigDefault")
        self.move_group.set_planning_time(1)
        # self.move_group.set_num_planning_attempts(10)
        self.tolerance = [0.1, 0.1, 0.1, 0.75, 0.75, 0.75]
        self.move_group.clear_pose_targets()


    def get_current_position(self):
        return self.move_group.get_current_pose().pose.position

    def get_current_orientain(self):
        # a = self.move_group.get_current_pose().pose.orientation  # return orientation in quaternions
        # orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])) - 2 * np.pi) % (2 * np.pi)
        return self.move_group.get_current_rpy()  # orien  # (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])))

    def go_home(self):
        # home = self.move_group.get_current_joint_values()   
        is_reached=self.move_group.go(self.home, wait=True)
        self.move_group.stop()
        if(is_reached):
            print('Home position reached')
        else:
            print('FAILED to reach home')

    def go_to_pose_goal(self, pose, orientaion, joints=None, links=None):
        """send position and orientaion of the desired point
        pose - x,y,z poistion - in world frame
        orientaion - roll, pitch, yaw position - in world frame
        return true if the movement succeeded and reach at the desired accuracy
        """
        # orientaion = self.get_current_orientain()
        # orientaion = [orientaion[0], orientaion[1], orientaion[2]]
        pose_goal = pose + orientaion
        self.move_group.set_pose_target(pose_goal)
        # self.move_group.set_position_target(pose)
        ind = 1
        if joints is None:
            joints = ["revolute"] * (len(self.move_group.get_active_joints())-2)
        if links is None:
            links = [0.1] * (len(self.move_group.get_active_joints())-2)
        tic = rospy.get_time()
        # we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)  # return true if succeed false if not
        if not plan:
            plan = self.move_group.go(wait=True)  # sometimes arrives but not in timeout
        toc = rospy.get_time()
        sim_time = round(toc - tic, 3)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if plan:
            ind = self.indices_calc(joints, links)
        orientaion = (np.asarray(orientaion)-2 * np.pi) % (2 * np.pi)
        goal = [pose[0], pose[1], pose[2], orientaion[0], orientaion[1], orientaion[2]]
        pos = self.get_current_position()
        orien = self.get_current_orientain()
        current = [pos.x, pos.y, pos.z, orien[0], orien[1], orien[2]]
        return plan, sim_time, ind


    def indices_calc(self, joints, links):
        try:
            # ignoring the final joint which is a roll
            cur_pos = self.move_group.get_current_joint_values()
            jacobian = np.delete(self.move_group.get_jacobian_matrix(cur_pos), -1, 1)
            cur_pos = np.asarray(cur_pos)
            # Jacobian singular values (~eighen values)
            j_ev = np.linalg.svd(jacobian, compute_uv=False)
            # Manipulability index
            mu = round(np.product(j_ev), 3)
            # Joint Mid-Range Proximity
            z = self.mid_joint_proximity(cur_pos, joints, links)
            return mu, np.diag(z), jacobian, cur_pos
        except:
            # if there numeric error like one of the values is NaN or Inf or divided by zero
            return -1, 1, np.asarray([-1]*len(joints)), jacobian, cur_pos

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
        z = np.around(0.5*np.transpose(nor_dis)*w, 3)
        return z

class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=None, joints=None, joints_axis=None, rpy=None):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link, who is always the same(connected to the world and link1  - his joint is limited to 0)
        """
        if rpy is None:
            rpy = []
        if joints_axis is None:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if joints is None:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if links is None:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy = rpy
        self.weights = self.calc_weight()

    def calc_weight(self):
        """
            defining mass value for each link
        """
        cumulative_length = 0
        cumulative_weight = 0
        link_mass = []
        for l in self.links:
            cumulative_length += float(l)
            mass = cumulative_length * 7.3149 + 1.1755 - cumulative_weight
            link_mass.append(str(mass))
            cumulative_weight += mass
        if len(link_mass) < 6: 
            while len(link_mass) < 6:
                fictive_mass = 0
                link_mass.append(str(fictive_mass))
        return link_mass

    # def calc_weight(self):
    #     """
    #     this function calculate the weight of the links according to accumilated weight and length of arm
    #     :return: weigths- the weight [kg] of each link - list of strings  (from the 2nd link)
    #     """
    #     coeffs = [7.3149, 1.1755]  # the coeffs of the linear eauation (found according UR5 and motoman)
    #     weights = [0]  # the wieght of each link
    #     acc_length = 0  # accumelated length
    #     acc_weight = 0  # accumelated weight
    #     for link in self.links[1:]:
    #         acc_length = acc_length+float(link)
    #         link_mass.append(round(acc_length*coeffs[0]+coeffs[1]-acc_weight,2))
    #         acc_weight = acc_weight + weights[-1]
    #     while len(weights) < 7:
    #         weights.append(1)
    #     return [str(weight) for weight in weights]

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
        <robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
        <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/''' + str(self.links_number) + '''dof/transmission_''' + str(
            self.links_number) + '''dof.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />

        <link name="world" />
        
        <joint name="world_joint" type="fixed">
            <parent link="world" />
            <child link = "base_link" />
            <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
        </joint>

        <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
        <inertial>
          <mass value="${mass}" />
          <xacro:insert_block name="origin" />
          <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
            iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
            izz="${0.5 * mass * radius * radius}" />
        </inertial>
        </xacro:macro>

        <xacro:macro name="joint_limit" params="joint_type link_length ">
            <xacro:if value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${pi}" />
                <xacro:property name="joint_lower_limit" value="${-pi}" />
            </xacro:if>
            <xacro:unless value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${link_length}" />
                <xacro:property name="joint_lower_limit" value="${0}" />
            </xacro:unless>
        <limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
        </xacro:macro>

        <xacro:macro name="arm_robot" params="prefix ">'''

        inertia_parameters = '''
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="base_length"  value="11" /> 
        <xacro:property name="base_height"  value="1" />
        <xacro:property name="base_width"  value="0.5" />
        <xacro:property name="base_mass" value="44" />


            <!-- Inertia parameters -->
        <xacro:property name="link0_mass" value="7" />
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="''' + self.weights[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.weights[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.weights[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.weights[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.weights[5] + '''" />

        <xacro:property name="link0_radius" value="0.060" /> 
        <xacro:property name="link1_radius" value="0.049" />
        <xacro:property name="link2_radius" value="0.045" />
        <xacro:property name="link3_radius" value="0.040" />
        <xacro:property name="link4_radius" value="0.035" />
        <xacro:property name="link5_radius" value="0.030" />
        <xacro:property name="link6_radius" value="0.025" /> '''

        base_link = '''

        <!-- Base Link -->
        <link name="${prefix}base_link" >
          <visual>
                <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
            <geometry>
                    <box size="${base_width} ${base_length} ${base_height}"/> 
            </geometry>
          </visual>
          <collision>
                 <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
            <geometry>
                    <box size="${base_width} ${base_length} ${base_height}"/>  
            </geometry>
          </collision>
          <inertial>
            <mass value="${base_mass}" />
            <origin xyz="0.0 0.0 ${base_height/2}" />
            <inertia ixx="436.5" ixy="0"  ixz="0"
                              iyy="1.8" iyz="0"
                                      izz="436.5"/>
          </inertial>
         
        </link>

        <xacro:property name="joint0_type" value="prismatic" /> 
        <xacro:property name="joint0_axe" value="0 1 0" /> 
        <xacro:property name="link0_length" value="0.1" />

        <!--  joint 0   -->
        <joint name="${prefix}joint0" type="${joint0_type}">
          <parent link="${prefix}base_link" />
          <child link = "${prefix}link0" />
          <origin xyz="0.0 0 ${base_height}" rpy="0 0.0 0" />
          <axis xyz="${joint0_axe}" />
          <xacro:joint_limit joint_type="${joint0_type}" link_length="${base_length/2}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>

         <!--  link 0 - The base of the robot  -->
        <link name="${prefix}link0">
          <visual>
            <origin xyz="0 0 ${link0_radius} " rpy="0 0 0" /> 
            <geometry>
                <cylinder radius="${link0_radius}" length="${link0_length}"/>    
            </geometry>
          </visual>
          <collision>
             <origin xyz="0 0 ${link0_radius}" rpy="0 0 0" /> 
            <geometry>
                <cylinder radius="${link0_radius}" length="${link0_length}"  mass="${link0_mass}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${link0_radius}" length="${link0_length}" mass="${link0_mass}">
            <origin xyz="0.0 0.0 ${link0_radius}" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link> 


    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''
        <!-- Sprayer joint - fictive joint -->
        <joint name="fake_joint" type="revolute">
            <parent link="${prefix}link''' + str(self.links_number) + '''" />
            <child link = "camera_link" />
            <origin xyz="0.0  0.0 ${link''' + str(self.links_number) + '''_length}" rpy="0.0 0.0 0" />
            <axis xyz="0 0 1"/>
            <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
            <dynamics damping="0.0" friction="0.0"/>
        </joint>
        

        <!-- Sprayer -->
        <link name="camera_link">
          <collision>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
          </collision>
        <visual>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
        </visual>
          <xacro:cylinder_inertial radius="0.01" length="0.01" mass="0.01">
            <origin xyz="0.0 0.0 0.005" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>

        <!-- ee joint -->
        <joint name="${prefix}ee_fixed_joint" type="fixed">
          <parent link="camera_link" />
          <child link = "${prefix}ee_link" />
          <origin xyz="0.0  0.0 0.01" rpy="0.0 0.0 0" />
        </joint>

            
        <!-- ee link -->
        <link name="${prefix}ee_link">
          <collision>
            <geometry>
              <box size="0.01 0.01 0.01"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
          </collision>
        </link>

        
            <xacro:arm_transmission prefix="${prefix}" />
            <xacro:arm_gazebo prefix="${prefix}" />
            </xacro:macro>
            <xacro:arm_robot prefix=""/>
        </robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    @staticmethod
    def link_create(n):
        """link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity"""
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
        <link name="${prefix}link1">
          <visual>
            <origin xyz="0 0 ${link1_length / 2} " rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </visual>
          <collision>
             <origin xyz="0 0 ${link1_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
            <origin xyz="0.0 0.0 ${link1_length / 2}" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        else:
            link = link + '''<!-- link ''' + str(n) + '''   -->
        <link name="${prefix}''' + linkname + '''">
          <visual>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </visual>
          <collision>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
            <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        return link

    def calc_origin(self, n):
        # calc the origin of the link according to the previuos joint
        if self.joint_data[n - 1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['${1/2*pi} ', '0 ', '0 ']:  # links in the same directoin
                    return "0 -${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
            else:  # pitch
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same directoin
                    return "0 ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} ${link" + str(
                        n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same directoin
                    return " ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(
                        n - 1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1    -->
        <joint name="${prefix}joint1" type="${joint1_type}">
          <parent link="${prefix}link0" />
          <child link="${prefix}link1" />
          <origin xyz="0.0 0.0 ${link0_length+0.011}" rpy="0.0 0.0 0.0" />
          <axis xyz="${joint1_axe}"/>
          <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''   -->
        <joint name="${prefix}''' + jointname + '''" type="${''' + jointname + '''_type}">
          <parent link="${prefix}link''' + str(n - 1) + '''"/>
          <child link="${prefix}link''' + str(n) + '''" />
          <origin xyz="''' + orgin + '''" rpy="''' + rpy + '''"/>
          <axis xyz="${''' + jointname + '''_axe}"/>
          <xacro:joint_limit joint_type="${''' + jointname + '''_type}" link_length="${link''' + str(n) + '''_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        return joint

    @staticmethod
    def urdf_write(data, filename = str(datetime.now())):
        #path='/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/'
        fil = open(filename + '.urdf.xacro', 'w')
        fil.write(data)
        fil.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    @staticmethod
    def axis_calc(axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
                datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'

