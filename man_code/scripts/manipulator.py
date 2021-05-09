import roslaunch
import rospy
from geometry_msgs.msg import Twist
import rosgraph
# Moveit libs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# System Libs
import subprocess
import shlex
import os
import time
from math import pi
import datetime
from logging import warning
import numpy as np
import csv
from socket import error


class Ros(object):
    """ This class is for using easly ros commands"""
    def __init__(self):
        """if needed add publish and subcribe"""
        try:
            self.pathname = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/"  # /Tamir_Ws
            self.data_back = "-5"
        except ValueError:
            rospy.loginfo('Error occurred at init')  # shows warning message
            pass

    def start_launch(self, launch_name, launch_path, args=None):
        """Start launch file"""
        if args is None:
            args = []
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            path = self.pathname+launch_path+"/launch/"+launch_name+".launch"
            cli_args = [path, args]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], args)]
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            return launch
        except ValueError:
            rospy.loginfo('Error occurred at start launch function')
            pass

    @staticmethod
    def stop_launch(launch):
        try:
            launch.shutdown()
            return False
        except ValueError:
            rospy.loginfo('Error occurred at launch_stop function')
            pass

    @staticmethod
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


class MoveGroupPythonInterface(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        # moveit_commander.roscpp_initialize(sys.argv)
        #  Provides information such as the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.box_name = ''
        self.cylinder_name = ''
        self.move_group.set_goal_orientation_tolerance(0.01)
        self.move_group.set_goal_position_tolerance(0.05)
        # Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        # Getting Basic Information
        self.planning_frame = self.move_group.get_planning_frame()
        self.move_group.set_planner_id("RRTstarkConfigDefault")
        self.move_group.set_planning_time(10)
        # self.move_group.set_num_planning_attempts(10)
        self.tolerance = [0.1, 0.1, 0.1, 0.75, 0.75, 0.75]
        self.move_group.clear_pose_targets()

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
        accuracy = self.all_close(goal, current, self.tolerance)
        accuracy = True
        return accuracy and plan, sim_time, ind

    def add_obstacles(self, height=3.75, radius=0.1, pose=None, timeout=4):
        if pose is None:
            pose = [0.5, 0]
        plant_height = 0.75
        floor = {'name': 'floor', 'pose': [0, 0, height - plant_height - 0.01], 'size': (7, 7, 0.02)}
        # Adding Objects to the Planning Scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = floor['pose'][0]
        box_pose.pose.position.y = floor['pose'][1]
        box_pose.pose.position.z = floor['pose'][2]
        self.box_name = floor['name']
        self.scene.add_box(self.box_name, box_pose, size=floor['size'])
        # self.scene.attach_box('base_link', self.box_name)
        # add plant
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = self.robot.get_planning_frame()
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = pose[0]
        cylinder_pose.pose.position.y = pose[1]
        cylinder_pose.pose.position.z = floor['pose'][2]+plant_height/2
        self.cylinder_name = 'plant'
        self.scene.add_cylinder(self.cylinder_name, cylinder_pose, plant_height, radius)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    @staticmethod
    def all_close(goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats
        @param: actual     A list of floats
        @param: tolerance  A list of floats
        @returns: bool
        """
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance[index]:
                if index > 2:  # for angles
                    if abs(actual[index] - goal[index]) < 2*pi - tolerance[index] and\
                            abs(actual[index] - goal[index]) < pi - tolerance[index] and \
                            abs(actual[index] - goal[index]) < pi + tolerance[index]:  # 2 pi with tolerance
                        return False
                else:
                    return False
        return True

    def get_current_position(self):
        return self.move_group.get_current_pose().pose.position

    def get_current_orientain(self):
        # a = self.move_group.get_current_pose().pose.orientation  # return orientation in quaternions
        # orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])) - 2 * np.pi) % (2 * np.pi)
        return self.move_group.get_current_rpy()  # orien  # (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])))

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Ensuring Collision Updates Are Receieved
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are made, we wait until we see the
        # changes reflected in the ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        start = time.time()  # rospy.get_time()
        seconds = time.time()  # rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene. -Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            time.sleep(0.1)
            seconds = time.time()  # rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False


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
        this function calculate the weight of the links according to accumilated weight and length of arm
        :return: weigths- the weight [kg] of each link - list of strings  (from the 2nd link)
        """
        coeffs = [8.79055, 4.2928]  # the coeffs of the linear eauation (found according UR5 and motoman)
        weights = [0]  # the wieght of each link
        acc_length = 0  # accumelated length
        acc_weight = 0  # accumelated weight
        for link in self.links[1:]:
            acc_length = acc_length+float(link)
            weights.append(round(acc_length*coeffs[0]+coeffs[1]-acc_weight,2))
            acc_weight = acc_weight + weights[-1]
        while len(weights) < 7:
            weights.append(1)
        return [str(weight) for weight in weights]

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
  <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />

<link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link = "base_link" />
    <origin xyz="0 -1 0" rpy="0.0 0.0 0.0" />
  </joint>

  <xacro:include filename="$(find man_gazebo)/urdf/'''+str(self.links_number)+'''dof/transmission_'''+str(self.links_number)+'''dof.xacro" />
  <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />

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
        <xacro:property name="base_length" value="3.25"/>
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="link0_radius" value="0.060" /> 
            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="1.0" />
        <xacro:property name="link0_mass" value="40.7" />
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="''' + self.weights[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.weights[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.weights[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.weights[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.weights[5] + '''" />
        
        <xacro:property name="link1_radius" value="0.049" />
        <xacro:property name="link2_radius" value="0.045" />
        <xacro:property name="link3_radius" value="0.040" />
        <xacro:property name="link4_radius" value="0.035" />
        <xacro:property name="link5_radius" value="0.030" />
        <xacro:property name="link6_radius" value="0.025" /> '''
        base_link = '''

        	<!--   Base Link -->
    <link name="${prefix}base_link" >

      <collision>
			<origin xyz="0 0 ${base_length/2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${base_radius}" length="${base_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${base_radius}" length="${base_length}" mass="${base_mass}">
        <origin xyz="0.0 0.0 ${base_length/2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>
    
    <xacro:property name="joint0_type" value="prismatic" /> 
    <xacro:property name="joint0_axe" value="0 0 1" /> 
    <xacro:property name="link0_length" value="0.25" />
<!--  joint 0	-->
    <joint name="${prefix}joint0" type="${joint0_type}">
      <parent link="${prefix}base_link" />
      <child link = "${prefix}link0" />
      <origin xyz="0.0 ${base_radius} ${base_length + link0_radius}" rpy="${-pi/2} 0.0 0" />
      <axis xyz="${joint0_axe}" />
	  <xacro:joint_limit joint_type="${joint0_type}" link_length="${link0_length*6}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

<!--  link 0  -->
    <link name="${prefix}link0">
      <visual>
		<origin xyz="0 0 ${link0_radius} " rpy="0 0 0" /> 
        <geometry>
            <box size="0.1 0.1 0.2"/>
        </geometry>
      </visual>
      <collision>
		 <origin xyz="0 0 ${link0_radius}" rpy="0 0 0" /> 
        <geometry>
			 <box size="0.1 0.1 0.2"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${link0_radius}" length="${link0_length}" mass="${link0_mass}">
        <origin xyz="0.0 0.0 ${link0_length/2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>

    
    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''
         <!-- fake joint - the rotation about z axe of the camera is not important -->
        <joint name="fake_joint" type="revolute">
      <parent link="${prefix}link''' + str(self.links_number) + '''" />
      <child link = "camera_link" />
      <origin xyz="0.0  0.0 ${link''' + str(self.links_number) + '''_length}" rpy="0.0 0.0 0" />
      <axis xyz="0 0 1"/>
      <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
      <dynamics damping="0.0" friction="0.0"/>
        </joint>

    <!-- camera link -->
        <link name="camera_link">
          <collision>
            <geometry>
              <box size="0.01 0.01 0.01"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
          </collision>
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
            link = link + '''<!-- link ''' + str(n) + '''	-->
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
        if self.joint_data[n-1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['${1/2*pi} ', '0 ', '0 ']:  # links in the same directoin
                    return "0 -${link" + str(n-1) + "_radius} ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n-1) + "_radius} ${link" + str(n-1) + "_length}"
            else:  # pitch
                if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same directoin
                    return "0 ${link" + str(n-1) + "_radius+link" + str(n) + "_radius} ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same directoin
                    return " ${link" + str(n-1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(n-1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1	-->
    <joint name="${prefix}joint1" type="${joint1_type}">
      <parent link="${prefix}link0" />
      <child link="${prefix}link1" />
      <origin xyz="0.0 0.0 ${link0_length}" rpy="${pi/2} 0.0 0.0" />
      <axis xyz="${joint1_axe}"/>
	  <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''	-->
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
    def urdf_write(data, filename=str(datetime.datetime.now().minute)):
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


class HandleCSV(object):

    @staticmethod
    def save_data(data, file_name):
        """Save to csv format"""
        with open(file_name + ".csv", 'ab') as name:
            writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerows(data)

    def read_data(self, file_name):
        with open(file_name + ".csv", 'r') as _filehandler:
            csv_file_reader = csv.reader(_filehandler)
            data = []
            manip = []
            empty = True
            for row in csv_file_reader:
                while "" in row:
                    row.remove("")
                if len(row) > 0:
                    if len(row) == 1:
                        row = row[0].split(",")
                    data.append(row)
                    empty = False
                else:
                    if not empty:
                        manip.append(self.read_data_action(data))
                        data = []
                    empty = True
            manip.append(self.read_data_action(data))  # append the last session
            _filehandler.close()
            return manip

    @staticmethod
    def read_data_action(data):
        manip = map(list, zip(*data))
        manip_array_of_dict = []
        for i in range(0, len(manip) - 1, 2):
            manip_array_of_dict.append({"joint": manip[i], "axe": manip[i + 1]})
        return manip_array_of_dict


def main_move_group():
    rospy.init_node('move_group_interface1', anonymous=True)
    #Ros()
    manipulator = MoveGroupPythonInterface()
    # a = manipulator.move_group.get_jacobian_matrix(manipulator.move_group.get_current_joint_values() )
    # u = manipulator.manipulability_index(a)
    time.sleep(0.13)
    manipulator.add_obstacles(height=3.75)  # add floor
    z = 3
    poses = [[0.5, 0, z + 0.9], [0.2, 0, z + 0.9], [0.2, 0.0, z + 0.65], [0.2, 0, z + 0.4]]
    oriens = [[0, -3.14, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.36, 0]]
    # poses = [[0.5, 0, z + 0.9], [0.55, 0, z + 0.9], [0.55, 0, z + 0.95], [0.5, 0, z + 0.95], [0.45, 0, z + 0.95],
    #               [0.45, 0, z + 0.9], [0.45, 0, z + 0.85], [0.5, 0, z + 0.85], [0.55, 0, z + 0.85],
    #               [0.2, 0, z + 0.9], [0.25, 0, z + 0.9], [0.25, 0, z + 0.95], [0.2, 0, z + 0.95], [0.15, 0, z + 0.95],
    #               [0.15, 0, z + 0.9], [0.15, 0, z + 0.85], [0.2, 0, z + 0.85], [0.25, 0, z + 0.85],
    #               [0.2, 0.0, z + 0.65], [0.25, 0.0, z + 0.65], [0.25, 0.0, z + 0.7], [0.2, 0.0, z + 0.7], [0.15, 0.0, z + 0.7],
    #               [0.15, 0.0, z + 0.65], [0.15, 0.0, z + 0.6], [0.2, 0.0, z + 0.6], [0.25, 0.0, z + 0.6],
    #               [0.2, 0, z + 0.4],  [0.25, 0, z + 0.4],  [0.25, 0, z + 0.45],  [0.2, 0, z + 0.45], [0.15, 0, z + 0.45],
    #               [0.15, 0, z + 0.4],  [0.15, 0, z + 0.35],  [0.2, 0, z + 0.35],  [0.25, 0, z + 0.35]]
    # oriens = [[0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0],
    #            [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0],
    #            [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0],
    #            [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.5, 0],
    #            [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0],
    #            [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0],
    #            [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0],
    #            [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0]]
    results = []
    for i in range(len(poses)):
        pose = poses[i]
        orientaion = oriens[i]
        results.append(manipulator.go_to_pose_goal(pose, orientaion))
        time.sleep(0.5)
        # raw_input("press enter")
    return results


# def myhook():
#     print "shutdown time!"


# def assign_data(data, arm, confname):
#     """
#     Calculate the manipulaot indices(Manipulability, Joint Mid-Range Proximity)
#     if the manipulator succed and the time that take
#     :param data: array of the result of the configuration about each detection point
#     :param arm: which configuration
#     :return: array of the results
#     """
#     data_res = []
#     jacobian = []
#     curr_pos = []
#     mu = []   # Manipulability index
#     z = []    # Joint Mid-Range Proximity
#     arm_number = str(arm + 1 )
#     for j in data:
#         data_res.append(j[0])
#         if j[0]:
#             mu.append(j[2][0])
#             z.append(j[2][1].min())
#             jacobian.append(j[2][2].tolist())
#             curr_pos.append(j[2][3].tolist())
#         else:
#             mu.append(-1)
#             z.append(-1)
#             jacobian.append(-1)
#             curr_pos.append(-1)
#     mu = np.asarray(mu)
#     z = np.asarray(z)
#     # choose only the min values because those are the "worst grade"
#     try:
#         mu_min = mu[mu >= 0.0].min()
#     except:
#         mu_min = -16
#     try:
#         z_max = z[z >= 0.0].max()
#     except:
#         z_max = 16
#     return [arm_number, confname, data_res, str(mu_min), str(z_max)]


if __name__ == '__main__':
    main_move_group()