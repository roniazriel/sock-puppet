from __future__ import print_function
from arm_creation import UrdfClass, create_arm
from ros import Ros,MoveGroupPythonInterface 
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

# def myexcepthook(type, value, tb):
#     import subprocess
#     import shlex
#     print("Ros has crashed Hook!!!!!!!!!!!!!!!!!!!!!!!")
#     command = ("kill -9 `ps aux | grep ros | grep -v grep | awk '{print $2}'`")
#     try:
#         command = shlex.split(command)
#         ter_command_proc = subprocess.Popen(command, stdout=subprocess.PIPE, preexec_fn=os.setsid)
#         return ter_command_proc
#     except ValueError:
#         rospy.loginfo('Error occurred at ter_command function')  # shows warning message
#         pass

# sys.excepthook = myexcepthook

class simulation(object):

    def __init__(self,joint_types,joint_axis,links,arm_name):
        self.path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"
        if joint_types is None:
            self.dof=6
        else:
            self.dof= len(joint_types)
        self.arms=[]
        self.joint_types =joint_types
        self.joint_axis=joint_axis
        self.links = links
        self.arm_name = arm_name
        grapes_coords=  [[1.2,2.9,1.5],
                         [1.14,2.08,1.57],
                         [1.5,1.42,1.8],
                         [1.5,0.08,1.2],

                         [1.8,-0.15,1.2],
                         [1.43,-1.53,1.47],
                         [1.18,-1.42,1.6],
                         [1.1,-2.15,1.5],
                         [1.16,-3.2,1.7],
                         [1.8,-4,1.8]]

        self.grapes_positions = grapes_coords
        print("MoveGroupPythonInterface")
        self.manipulator_move = MoveGroupPythonInterface() 
        print("Ros")
        self.ros = Ros()
        

    def set_links_length(self, min_length=1, max_lenght = 2, link_min=0.1, link_interval=0.2, link_max=0.71):
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
            if sum(link) >= min_length and sum(link) <= max_lenght :
                links.append([str(x) for x in link])
        return links

    def create_urdf_from_csv(self, csv_name="all_configs6", folder="arms", num_of_group=2, min_length=1.4): # - create all the urdf for possible robotic models
        # read from csv file with all the possible configuration for manipulators
        base_path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/"
        configs = self.read_data(base_path+csv_name)
        # Create the urdf files
        data = []
        self.create_folder(base_path + str(self.dof) + "dof/"+ folder)
        links = self.set_links_length(min_length = min_length)
        index = 0
        for config in configs:
            for arm in config:
                random_links = random.sample(links,num_of_group)
                for link in random_links:
                    self.arms.append(create_arm(arm["joint"], arm["axe"], link, folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                    #self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                     #                                  path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"].replace(" ", ""), folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        print(data,"data")
        print("number of arms: ",len(data))
        #print(self.arms, "arms")

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


    # Generate URDF by given inputs
    def generate_urdf(self):
        if self.joint_types is None:
            self.joint_types =["roll","pris", "roll", "roll", "roll", "pitch"]
        if self.joint_axis is None:
            self.joint_axis =['z', 'z', 'y', 'y', 'y', 'z']
        if self.links is None:
            self.links = ['0.1', '0.5', '0.3', '0.5', '0.3', '0.1']

        self.dof = len(self.joint_types)
        arm = create_arm(self.joint_types, self.joint_axis, self.links)
        arm_name = arm["name"]
        print(arm_name)
        return arm_name


    # Launch URDF Model
    def launch_model(self, man="manipulator"):
        main_launch_arg = ["gazebo_gui:=true", "rviz:=false", "dof:=" + "6" + "dof", "man:=" + man]
        launch = self.start_launch("main", self.path, "man_gazebo", main_launch_arg)  # main launch file
        return launch

    def write_indicators_to_csv(self,db,pose_id,Time,plan_result,mu,z,jacobian,cur_pose,mu_roni):
        db = db.append({'Arm ID': self.arm_name,
                                    'Point number': pose_id,
                                    'Move duration': Time,
                                    'Sucsses': plan_result,
                                    'Manipulability - mu': mu,
                                    'Manipulability - jacobian': jacobian,
                                    'Manipulability - cur pose': cur_pose,
                                    'Manipulability - roni': mu_roni,
                                    'Mid joint proximity': z},ignore_index=True)
        return db
    
    
    def start_spray(self,joints,links,db):
        print('start_spray')
        pose_id = 1
        for goal in self.grapes_positions:
            db = self.go_pose(goal,joints,links,db,pose_id)
            pose_id +=1
        return db


    # spray 10 clusters and take measurments
    def performe_spray(self,db):
        joints = self.joint_types
        links =self.links
        spray = Spray(arm_name=self.arm_name)
        points_db = spray.start_spray(joints,links,db)
        print(points_db,"performe_spray")

        return points_db


    def simulate(self,db):
        try:
           #joints, joint_parent_axis, links, arm_name = self.generate_urdf(None, None, None ,None, None)
            start_sim = time.time()
            print("launch_model ")
            launch = self.launch_model(man=self.arm_name)
            launch_time = time.time()
            time.sleep(10)
            
            ros = Ros()
            # clean ros log file
            ros.ter_command("rosclean purge -y")
            # check if there is roscore running if there is stop it
            roscore = ros.checkroscorerun()
            if roscore:
                ros.ter_command("kill -9 " + str(roscore))
            # start roscore
            ros.ros_core_start()
            init_node('arl_python', anonymous=True)
            
            print("performe_spray ")
            simulation_db = self.performe_spray(db)
            print("after performe_spray ")
            end_spary = time.time()
            launch.shutdown() # kill proccess
            
            roscore = ros.checkroscorerun()
            if roscore:
                ros.ter_command("kill -9 " + str(roscore))


            rospy.signal_shutdown("Finish Spraying")
            while not rospy.is_shutdown():
                pass
            print("Ros is Shutdown")

            shutdown_time = time.time()
            print("launch time: ",launch_time - start_sim)
            print("spary with movement time: ",end_spary - launch_time)
            print("shutdown time: ",shutdown_time - end_spary)
            print("total time with reaching clusters:" ,shutdown_time - start_sim)
            print(simulation_db,"simulate")
            return simulation_db
        except:
            print("Ros has crashed- in simulate function!!!!!!!!!!!!!!!!!!!!!!!")
            command = ("kill -9 `ps aux | grep ros | grep -v grep | awk '{print $2}'`")
            self.ter_command(command)

    def create_urdf_file (self, joint_types,joint_axis,links,arm_name):
        ''' Creating 1 URDF '''
        if joint_types is None:
            joint_types=["roll","pris","roll","roll","roll", "pris"]

        if joint_axis is None:
            joint_axis= ['z', 'z','y', 'y', 'y','y']

        if links is None:
            links =['0.1', '0.5','0.5', '0.5', '0.3','0.1']
        if arm_name is None:
            arm_name=None

        self.generate_urdf()

    def calc_links_options(min_length=1, max_lenght = 2, link_min=0.1, link_interval=0.2, link_max=0.71):
        ''' Calculate links lenghts options amount''' 
        self.set_links_length()
        print("Links Lenghts options amount: ",len(links))


def multiple_simulations(directory= '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/', num_to_write=1000, num_to_start=999):
    simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Sucsses", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Mid joint proximity",""])
    #directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
    #file_number=999
    print("here")
    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f):
            head,arm_name = os.path.split(f[0:-11])
            configuration = arm_name.split("_")
            configuration = configuration[1:]
            joint_types=[]
            joint_axis=[]
            links=[]
            for i in range(0,len(configuration)-3,4):
                joint_types.append(configuration[i])
                joint_axis.append(configuration[i+1])
                links.append(configuration[i+2]+"."+configuration[i+3])
            print("here 2")
            sim = simulation(joint_types,joint_axis,links, arm_name) 
            print("simulate")   
            simulation_db = sim.simulate(simulation_db)          
            print(simulation_db)
            file_number +=1
            if file_number % num_to_write == 0 :
                pd.DataFrame(simulation_db).to_csv(os.environ['HOME'] + "/catkin_ws/src/sock-puppet/" +"results/sim_results"+str(file_number)+".csv")

if __name__ == '__main__':
    ''' Runing simulation- Go through all URDF files in 6dof/arms folder
        all simulations data is saved in sim_results file
        all URDFs has to be saved in a configuration name format
    '''
    multiple_simulations()
    # simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Sucsses", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Mid joint proximity",""])
    # directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
    # file_number=999
    # for filename in os.listdir(directory):
    #     f = os.path.join(directory, filename)
    #     # checking if it is a file
    #     if os.path.isfile(f):
    #         head,arm_name = os.path.split(f[0:-11])
    #         configuration = arm_name.split("_")
    #         configuration = configuration[1:]
    #         joint_types=[]
    #         joint_axis=[]
    #         links=[]
    #         for i in range(0,len(configuration)-3,4):
    #             joint_types.append(configuration[i])
    #             joint_axis.append(configuration[i+1])
    #             links.append(configuration[i+2]+"."+configuration[i+3])

    #         sim = simulation(joint_types,joint_axis,links, arm_name)    
    #         simulation_db = sim.simulate(simulation_db)          
    #         print(simulation_db)
    #         file_number +=1
    #         if file_number % 1000 == 0 :
    #             pd.DataFrame(simulation_db).to_csv(os.environ['HOME'] + "/catkin_ws/src/sock-puppet/" +"results/sim_results"+str(file_number)+".csv")


    ''' Creating the folder that contains all the sampled robotic arms (URDFs)
    '''
    # sim = simulation(None, None, None ,None)  
    # sim.create_urdf_from_csv(csv_name="all_configs6_part1", folder="arms1", num_of_group=10, min_length=1.65)

    ''' Creating 1 URDF
    '''
    # joint_types=["roll","pris","roll","roll","roll", "pris"]
    # joint_axis= ['z', 'z','y', 'y', 'y','y']
    # links =['0.1', '0.5','0.5', '0.5', '0.3','0.1']
    # arm_name=None
    # sim = simulation(joint_types, joint_axis, links ,arm_name)  
    # sim.generate_urdf()
 
    ''' Calculate links lenghts options amount
    '''
    # sim = simulation(None, None, None ,None)  
    # links = sim.set_links_length()
    # print("Links Lenghts options amount: ",len(links))