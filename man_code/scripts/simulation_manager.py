from __future__ import print_function
from arm_creation import UrdfClass, create_arm
import roslaunch
import os
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
from itertools import product
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Spray():
    def __init__(self):
        try:
          from math import pi, tau, dist, fabs, cos
        except: # For Python 2 compatibility
          from math import pi, fabs, cos, sqrt
          tau = 2.0*pi
          def dist(p, q):
            return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('spray', anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.plan_results=[]
        self.move_duration=[]
        self.performance_indicators=[]
        grapes_coords=  [[1.2,2.9,1.5],
                         [1.14,2.08,1.57],
                         [1.51,1.42,1.85],
                         [1.26,0.08,2.06],

                         [1.05,-0.15,1.3],
                         [1.43,-1.53,1.47],
                         [1.18,-1.42,1.6],
                         [1.1,-3.15,1.5],
                         [1.16,-4.2,1.7],
                         [1.4,-4.5,1.8]]
        self.grapes_positions = grapes_coords
        self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"  



    def go_home(self):
        # home = self.move_group.get_current_joint_values()   
        is_reached=self.move_group.go(self.home, wait=True)
        self.move_group.stop()
        if(is_reached):
            print('Home position reached')
        else:
            print('FAILED to reach home')

    # new_pos = move_group.get_current_joint_values()
    # new_pos[0] = 1

    def go_pose(self,goal, joints, links):
        spray_offset_x = 0.35
        vine_offset_x = 0.25
        start = time.time()
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1
        pose_goal.position.x = goal[0] - spray_offset_x- vine_offset_x
        pose_goal.position.y = goal[1]
        pose_goal.position.z = goal[2]

        self.move_group.set_pose_target(pose_goal)
        self.plan_result = self.move_group.go(wait=True)
        self.plan_results.append(self.plan_result)
        self.move_group.stop()
        end = time.time()
        print('Time:',end - start)
        self.move_duration.append(end - start)
        self.move_group.clear_pose_targets()

        if(self.plan_result):
            print("Success - A plan was created")
            performance = self.indices_calc(joints, links)
            self.performance_indicators.append(performance)
            time.sleep(5) # spraying time
            print('Goal position reached')
        else:
            print("Failed - The goal is out of reach")
            self.performance_indicators.append(" ")
            time.sleep(3)# sad

        print(self.move_duration)
        print(self.plan_results)

        pd.DataFrame(self.move_duration).to_csv(self.path +"results/move_duration.csv", header=None, index=None)
        pd.DataFrame(self.plan_results).to_csv(self.path +"results/plan_results.csv", header=None, index=None)
        pd.DataFrame(self.grapes_positions).to_csv(self.path +"results/grapes_coords.csv", header=None, index=None)
        pd.DataFrame(self.performance_indicators).to_csv(self.path +"results/performance_indicators.csv", header=None, index=None)

    
    def start_spray(self,joints,links):
        print('start_spray')
        # self.home= self.move_group.get_current_pose()
        for goal in self.grapes_positions:
            self.go_pose(goal, joints,links)
            # self.go_home()

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


class simulation(object):

    def __init__(self):
        self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"
        self.dof=6
        self.arms=[]

        
    def ter_command(self,command):
    	#"""Write Command to the terminal"""
        try:
            command = shlex.split(command)
            ter_command_proc = subprocess.Popen(command, stdout=subprocess.PIPE, preexec_fn=os.setsid)
            return ter_command_proc
        except ValueError:
            rospy.loginfo('Error occurred at ter_command function')  # shows warning message
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

    @staticmethod
    def create_folder(name):
        if not os.path.exists(name):
            mkdir(name)
        return name

    def set_links_length(self, min_length=1.4, link_min=0.1, link_interval=0.2, link_max=0.71):
    # set all the possible links lengths in the defind interval
    # :param min_length: the minimum length of all the links
    # :param link_min: minimum length of a link
    # :param link_interval: interval between joints lengths
    # :param link_max: maximun length of a link
    # :return: links: all the lengths combinations in the minimum length - list of strings
        links = []
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        links_length = [[0.1] + list(tup) for tup in
                        list(product(lengths_2_check, repeat=(self.dof - 1)))]
        for link in links_length:
            if sum(link) > min_length:
                links.append([str(x) for x in link])
        return links

    def create_urdf_from_csv(self, csv_name="all_configs", folder="arms"):
        # read from csv file with all the possible configuration for manipulators
        base_path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/"
        configs = self.read_data(base_path+csv_name)
        # Create the urdf files
        data = []
        self.create_folder(base_path + str(self.dof) + "dof/"+ folder)
        links = self.set_links_length()
        index = 0
        folder_num = 0
        for config in configs:
            for arm in config:
                for link in links:
                    self.arms.append(create_arm(arm["joint"], arm["axe"], link, folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"].replace(" ", ""), folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        print(data,"data")
        print(arms, "arms")
        # self.save_json("created arms", data)

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


    # Generate URDF
    def generate_urdf(self,interface_joints, joint_parent_axis, links, file_name, folder):
        if interface_joints is None:
            interface_joints =["roll","roll", "roll", "roll", "roll", "roll"]
        if joint_parent_axis is None:
            joint_parent_axis =['z', 'y', 'y', 'y', 'y', 'z']
        if links is None:
            links = ['0.1', '0.7', '0.4', '0.7', '0.1', '0.6']
        if file_name is None:
            file_name = "test_arm"
        if folder is None:
            folder = "folder"

        self.dof = len(interface_joints)
        arm = create_arm(interface_joints, joint_parent_axis, links, folder)
        arm_name = arm["name"]
        print(arm_name)
        return interface_joints, joint_parent_axis, links


    # Launch URDF Model
    def launch_model(self):
        main_launch_arg = ["gazebo_gui:=false", "rviz:=true", "dof:=" + "6" + "dof", "man:= manipulator"]
        launch = self.start_launch("main", self.path, "man_gazebo", main_launch_arg)  # main launch file
        return launch


    # spray 10 clusters and take measurments
    def performe_spray(self,joints,links):
        spray = Spray()
        spray.start_spray(joints,links)


    def simulate(self):
        joints, joint_parent_axis, links = self.generate_urdf(None, None, None ,None, None)
        launch = self.launch_model()
        self.performe_spray(joints,links)
        launch.shutdown() # kill proccess


if __name__ == '__main__':
    simulation = simulation()
    # simulation.simulate()
    # links = simulation.set_links_length()
    # print("number of options is: ", len(links))
    # 6dof_configs = pd.read_csv("/home/roni/catkin_ws/src/sock-puppet/man_gazebo/urdf/all_configs.csv", header=None, nrows=68359)
    # print(df_6dof)

    # file = simulation.read_data('/home/roni/catkin_ws/src/sock-puppet/man_gazebo/urdf/all_configs')
    # print(file)

    #simulation.create_urdf_from_csv()

    

