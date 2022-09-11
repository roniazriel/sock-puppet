import os
import time
import moveit_msgs.msg
import geometry_msgs.msg
import pandas as pd
import math
import re
from ros import Ros, MoveGroupPythonInterface, UrdfClass
from datetime import datetime
from os import environ, listdir, path, mkdir, rename
import numpy as np
from itertools import product
from time import sleep
from rospy import init_node
from rosnode import get_node_names
import getpass
import sys
import json
import rospy
import csv
import random

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

from os.path import expanduser 
home = expanduser("~") 

class Simulator(object):

    def __init__(self, arm_name, dof=6,folder = 'arms',create=False, arms=None ):
        # if arms is None:
        #   arms = []
        self.path= os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"
        self.dof = dof
        self.folder = folder
        
        self.ros = Ros()  # for work with Ros
        self.ros.ros_core_start()
        init_node('arl_python', anonymous=True)
        
        self.arm_control = 0
        self.arms = []
        self.json_data = []
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv()
        # else:
            # if not arms:
            #     self.arms_exist()
            # else:
            #     self.arms = arms

        # desired positions of the EE in world frame
        # grapes_cords=  [[1.1,2.9,1.35],
        #                 [1.3,2.08,1.57],
        #                 [0.7,1.42,1.8],
        #                 [1.2,0.08,1.2],

        #                 [0.7,-0.15,1.2],
        #                 [1.15,-1.53,1.47],
        #                 [1.2,-1.42,1.6],
        #                 [1.3,-2.15,1.8],
        #                 [0.95,-3.2,1.68],
        #                 [1.3,-4,1.2]]

        # grapes_cords=  [[1.1,0,1.35],
        #                 [1.3,0,1.57],
        #                 [0.7,0,1.8],
        #                 [1.2,0,1.2],

        #                 [0.7,0,1.2],
        #                 [1.15,0,1.47],
        #                 [1.2,0,1.6],
        #                 [1.3,0,1.8],
        #                 [0.95,0,1.68],
        #                 [1.3,0,1.2]]

        grapes_cords = [[1.2, 0, 1.2],
                [1.05, 0, 1.28],
                [1.1, 0, 1.35],
                [0.9, 0, 1.47],
                [1.2, 0, 1.5],
                [1.05, 0, 1.6],
                [0.85, 0, 1.7],
                [1.1, 0, 1.75],
                [1.2, 0, 1.8],
                [0.7, 0, 1.8]]

        self.grapes_positions = grapes_cords
        self.save_name = 'results_file' + datetime.now().strftime("%d_%m_") + str(dof) + "dof_" \
                        + str(self.dof) + "d_"
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof" , "man:=" +str(arm_name)]
        self.main = self.ros.start_launch("main", self.path, "man_gazebo", main_launch_arg)  # main launch file
        sleep(0.1)
        import moveit_commander
        import moveit_msgs.msg
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        pos = self.manipulator_move.get_current_position()
        orien = self.manipulator_move.get_current_orientain()
        self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], [orien[0], orien[1], orien[2]])
        
    @staticmethod
    def save_json(name="data_file", data=None):
        with open(name + ".json", "a") as write_file:
            json.dump(data, write_file, indent=2)

    @staticmethod
    def load_json(name="data_file"):
        with open(name + ".json", "r") as read_file:
            return json.load(read_file)

    def arms_exist(self):
        path = environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/" + str(self.dof) \
               + "dof/" + self.folder
        for fil in listdir(path):
            fol = self.folder.split("/")
            data = self.get_urdf(name=fil.replace(".urdf.xacro", ""))
            print(self.create_arm(data[0], data[1], data[2], fol[0]),"arm")
            self.arms.append(self.create_arm(data[0], data[1], data[2], fol[0]))
            # self.arms.append({"name": fil.replace(".urdf.xacro", ""), "folder": fol[0]})

    @staticmethod
    def get_urdf(name=""):
        joints = ["roll"]
        prev_axe = ["z"]
        link_length = ["0.1"]
        arm = name.split("_")
        for a in range(3, len(arm) - 1):
            if a % 3 == 0:
                joints.append(arm[a][1:])
            elif a % 3 == 1:
                prev_axe.append(arm[a])
            elif a % 3 == 2:
                link_length.append(arm[a] + "." + arm[a + 1][:1])
        return [joints, prev_axe, link_length]


    #@staticmethod
    def create_arm(self,interface_joints, joint_parent_axis, links, folder):
        """create the desired arm
            interface_joints- roll,pitch,yaw or prismatic
                             roll - revolute around own Z axe
                             pitch - revolute that not roll
                             pris - prismatic along
            links - length of links
            joint_parent_axis - the axe, in the parent frame, which each joint use
        """
        joints = []
        joint_axis = []
        rpy = []
        file_name = ""
        rolly_number = -1
        pitchz_number = 1
        prisy_number = -1
        for i in range(len(joint_parent_axis)):
            file_name += "_" + interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + links[i].replace(".", "_")
            if interface_joints[i].replace(" ", "") == "roll":
                joints.append("revolute")
                joint_axis.append('z')
                if joint_parent_axis[i].replace(" ", "") == "y":
                    # rpy.append(['${-pi/2} ', '0 ', '0 '])
                    rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                    rpy.append([rolly_rot, '0 ', '0 '])
                    rolly_number = rolly_number * -1
                elif joint_parent_axis[i].replace(" ", "") == "x":
                    rpy.append(['0 ', '${pi/2} ', '0 '])
                elif joint_parent_axis[i].replace(" ", "") == "z":
                    rpy.append(['0 ', '0 ', '0 '])
            elif interface_joints[i].replace(" ", "") == "pitch":
                joints.append("revolute")
                joint_axis.append('y')
                if joint_parent_axis[i].strip() == "y":
                    rpy.append(['0 ', '0 ', '0 '])
                elif joint_parent_axis[i].strip() == "x":
                    rpy.append(['0 ', '0 ', '${-pi/2} '])
                elif joint_parent_axis[i].strip() == "z":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    pitchz = '${' + str(pitchz_number) + '/2*pi} '
                    rpy.append([pitchz, '0 ', '0 '])
                    pitchz_number = pitchz_number * -1
            elif interface_joints[i].replace(" ", "") == "pris":
                joints.append("prismatic")
                joint_axis.append('z')
                if joint_parent_axis[i].strip() == "y":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    prisy = '${' + str(prisy_number) + '/2*pi} '
                    rpy.append([prisy, '0 ', '0 '])
                    prisy_number = prisy_number * -1
                elif joint_parent_axis[i].strip() == "x":
                    rpy.append(['0 ', '${-pi/2} ', '0 '])
                elif joint_parent_axis[i].strip() == "z":
                    rpy.append(['0 ', '0 ', '0 '])
        arm = UrdfClass(links = links, joints=joints, joints_axis=joint_axis, rpy=rpy)
        return {"arm": arm, "name": file_name, "folder": folder}

    def create_urdf_from_csv(self, csv_name="all_configs4", folder="arms", num_of_group=None, min_length=1.4): # - create all the urdf for possible robotic models
        # read from csv file with all the possible configuration for manipulators
        # num_of_group =  how many links configuration to sample for each joints configurations
        base_path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/"
        configs = self.read_data(base_path+csv_name)
        # Create the urdf files
        data = []
        self.ros.create_folder(base_path + str(self.dof) + "dof/"+ folder)
        links = self.set_links_length(min_length = min_length)
        index = 0
        for config in configs:
            for arm in config:
                if num_of_group is None:
                    links_configs = links
                else: 
                    links_configs = random.sample(links,num_of_group)
                for link in links_configs:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                      path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"].replace(" ", ""), folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        print(data,"data")
        print("number of arms: ",len(data))
        #print(self.arms, "arms")

    def set_links_length(self, min_length=1.4, max_lenght = 2, link_min=0.1, link_interval=0.2, link_max=0.71):
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
        print("number of links: ",len(links))
        return links

    #@staticmethod
    def read_data(self,file_name):
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

    #@staticmethod
    def read_data_action(self,data):
        manip = map(list, zip(*data))
        manip_array_of_dict = []
        for i in range(0, len(manip) - 1, 2):
            manip_array_of_dict.append({"joint": manip[i], "axe": manip[i + 1]})
        return manip_array_of_dict


    # Generate URDF by given inputs
    def generate_urdf(self,joint_types,joint_axis,links):
        if joint_types is None:
            joint_types =["roll","pris", "roll", "roll", "roll", "pitch"]
        if joint_axis is None:
            joint_axis =['z', 'z', 'y', 'y', 'y', 'z']
        if links is None:
            links = ['0.1', '0.5', '0.3', '0.5', '0.3', '0.1']

        arm = self.create_arm(self.joint_types, self.joint_axis, self.links)
        arm_name = arm["name"]
        print(arm_name)
        return arm_name


    # Launch URDF Model
    def launch_model(self, man="manipulator"):
        main_launch_arg = ["gazebo_gui:=true", "rviz:=false", "dof:=" + "6" + "dof", "man:=" + man]
        launch = self.start_launch("main", self.path, "man_gazebo", main_launch_arg)  # main launch file
        return launch

    def write_indicators_to_csv(self,db,pose_id,arm_name,Time,plan_result,mu,z,jacobian,cur_pose,mu_roni,z_max,z_sum, z_sum_all):
        db = db.append({'Arm ID': arm_name,
                                    'Point number': pose_id,
                                    'Move duration': Time,
                                    'Success': plan_result,
                                    'Manipulability - mu': mu,
                                    'Manipulability - jacobian': jacobian,
                                    'Manipulability - cur pose': cur_pose,
                                    'Manipulability - roni': mu_roni,
                                    'Mid joint proximity': z,
                                    'Max Mid joint proximity':z_max,
                                    'Sum Mid joint proximity':z_sum,
                                    'Sum Mid joint proximity- all joints':z_sum_all},ignore_index=True)
        return db
    
    
    def start_spray(self,arm_name,joints,links,db):
        print('start_spray')
        pose_id = 1
        for goal in self.grapes_positions:
            print("pose_id: ", pose_id)
            db = self.go_pose(goal,arm_name,joints,links,db,pose_id)
            pose_id +=1
        return db

    def go_pose(self,goal,arm_name, joints, links,db,pose_id):
        try:
            spray_offset_x = 0.2
            # the center of the platform is in (1.5 meters - vine_offset_x)
            #vine_offset_x = 0.8
            start = time.time()

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.y = 0.707
            # pose_goal.orientation.w = 1
            pose_goal.position.x = goal[0] - spray_offset_x #- vine_offset_x
            pose_goal.position.y = goal[1]
            pose_goal.position.z = goal[2]

            self.manipulator_move.move_group.set_pose_target(pose_goal)

            # with movement:
            self.plan_result = self.manipulator_move.move_group.go(wait=True)
            self.manipulator_move.move_group.stop()

            # only plan- without reaching the target
            # self.plan_result = self.move_group.plan()
            # self.plan_result = not(self.plan_result.joint_trajectory.points == [])

            end = time.time()
            Time = end-start
            self.manipulator_move.move_group.clear_pose_targets()

            if(self.plan_result):
                print("Success - A plan was created")
                mu,z,jacobian,cur_pose,mu_roni , z_max, z_sum, z_sum_all= self.indices_calc(joints, links)
                time.sleep(0.1) # spraying time
                print('Goal position reached')
            else:
                print("Failed - The goal is out of reach")
                mu,z,jacobian,cur_pose,mu_roni,z_max,z_sum,z_sum_all = "","","","","","","",""
                time.sleep(0.1) # pause
            print(self.plan_result,"self.plan_result")
            pose_db = self.write_indicators_to_csv(db,pose_id,arm_name,Time,self.plan_result,mu,z,jacobian,cur_pose,mu_roni,z_max,z_sum, z_sum_all) 
            return pose_db
        except:
            print("Ros has crashed in go pose function!!!!!!!!!!!!!!!!!!!!!!!")
            command = ("kill -9 `ps aux | grep ros | grep -v grep | awk '{print $2}'`")
            self.ter_command(command)

    def indices_calc(self, joints, links):
        try:
            # ignoring the final joint which is a roll 
            cur_pos = self.manipulator_move.move_group.get_current_joint_values()
            jacobian = np.delete(self.manipulator_move.move_group.get_jacobian_matrix(cur_pos), -1, 1)
            cur_pos = np.asarray(cur_pos)
            # Jacobian singular values (~eighen values)
            j_ev = np.linalg.svd(jacobian, compute_uv=False)
            print("j",j_ev)
            # Manipulability index
            mu_roni = 0
            # mu_roni= math.sqrt(np.linalg.det(np.matmul(jacobian,(np.transpose(jacobian)))))
            # print("mu roni")
            mu = round(np.product(j_ev), 3)
            print("mu",mu)
            # Joint Mid-Range Proximity
            z = self.mid_joint_proximity(cur_pos, joints, links)
            print("z")
            # 2 first joints are identical between the arms
            z_cur = np.delete(np.diag(z), [0,1])
            print("z_cur",z_cur)
            z_max = z_cur.max()
            z_sum = z_cur.sum()
            z_sum_all = np.diag(z).sum()
            return mu, np.diag(z), jacobian, cur_pos,mu_roni, z_max, z_sum, z_sum_all
        except:
            print("indices_calc - exception")
            # if there numeric error like one of the values is NaN or Inf or divided by zero
            return -1,np.asarray([-1]*len(joints)), jacobian, cur_pos , -1,-1, -1, -1


    def mid_joint_proximity(self,cur_pos, joints, link_length):
        try:
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
        except:
            print("mid joint -execpt")


    def simulate(self,db, arm_name,joints,links):
        try:
            start_sim = time.time()
            launch_time = time.time()
            simulation_db = self.start_spray(arm_name,joints,links,db)
            end_spary = time.time()
            self.main.shutdown() # kill proccess
            
            roscore = self.ros.checkroscorerun()
            if roscore:
                self.ros.ter_command("kill -9 " + str(roscore))

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
            ros = Ros()
            print("Ros has crashed- in simulate function!!!!!!!!!!!!!!!!!!!!!!!")
            command = ("kill -9 `ps aux | grep ros | grep -v grep | awk '{print $2}'`")
            ros.ter_command(command)


def one_at_a_time (dof,arm_name,simulation_db,joint_types,links):
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

    sim = Simulator( dof=dof, arm_name =arm_name) 

    print("simulate")   
    simulation_db = sim.simulate(simulation_db, arm_name=arm_name, joints=joint_types , links=links)
    import os.path
    if(os.path.isfile('/home/ar1/catkin_ws/src/sock-puppet/results/4dof_valid_results.csv')):
        pd.DataFrame(simulation_db).to_csv('/home/ar1/catkin_ws/src/sock-puppet/results/4dof_valid_results.csv', mode='a', index= True, header=False)
    else:
        pd.DataFrame(simulation_db).to_csv('/home/ar1/catkin_ws/src/sock-puppet/results/4dof_valid_results.csv', index= True)
    # pd.DataFrame(simulation_db).to_csv(os.environ['HOME'] + "/catkin_ws/src/sock-puppet/" +"results/test_results"+str(file_number)+".csv")          
    # print(simulation_db)

def multiple_simulations(directory= home+'/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/', num_to_write=1000, num_to_start=999):
    simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Sucsses", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Mid joint proximity",""])
    #directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
    file_number=999
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

            # ros = Ros()
            # # clean ros log file
            sim = Simulator( dof=6,arm_name =arm_name) 

            print("simulate")   
            # ros.ter_command("rosclean purge -y")
            # # check if there is roscore running if there is stop it
            # roscore = ros.checkroscorerun()
            # if roscore:
            #     ros.ter_command("kill -9 " + str(roscore))
            
            # start roscore
            # ros.ros_core_start()
            # init_node('arl_python', anonymous=True)

            sim = Simulator( dof=6,arm_name =arm_name) 

            print("simulate")   
            simulation_db = sim.simulate(simulation_db, arm_name=arm_name, joints=joint_types , links=links)          
            print(simulation_db)

            file_number +=1
            if file_number % num_to_write == 0 :
                pd.DataFrame(simulation_db).to_csv(os.environ['HOME'] + "/catkin_ws/src/sock-puppet/" +"results/sim_results"+str(file_number)+".csv")

def set_links_options(min_length=1.4, max_lenght = 2, link_min=0.1, link_interval=0.2, link_max=0.71, dof=6):
# set all the possible links lengths in the defind interval
# :param min_length: the minimum length of all the links
# :param link_min: minimum length of a link
# :param link_interval: interval between joints lengths
# :param link_max: maximun length of a link
# :return: links: all the lengths combinations in the minimum length - list of strings
    links = []
    lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
    links_length = [[0.1] + list(tup) for tup in
                    list(product(lengths_2_check, repeat=(dof - 1)))]
    for link in links_length:
        if sum(link) >= min_length and sum(link) <= max_lenght :
            links.append([str(x) for x in link])
    return links

def calc_links_options(min_length=1.4, max_lenght = 2, link_min=0.1, link_interval=0.2, link_max=0.71, dof=6):
    ''' Calculate links lenghts options amount''' 
    links = set_links_options(min_length, max_lenght, link_min, link_interval, link_max, dof)
    print(type(links))
    print("Links Lenghts options amount: ",len(links))



if __name__ == '__main__':
    ''' Runing simulation- Go through all URDF files in 6dof/arms folder
        all simulations data is saved in sim_results file
        all URDFs has to be saved in a configuration name format
    '''
    simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Success", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Manipulability - roni","Mid joint proximity",
                                            "Max Mid joint proximity","Sum Mid joint proximity","Sum Mid joint proximity- all joints"])
    directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/'
    dof = 4

    print(sys.argv[0])
    print(sys.argv[1])
    arm_name_urdf = sys.argv[1] # var1
    file_number = sys.argv[2] # var2
    print(file_number, "file_number")
    arm_name = re.sub('\.urdf$', '', arm_name_urdf)
    print("arm name",arm_name)
    #head,arm_name = os.path.split(f[0:-11])
    configuration = arm_name.split("_")
    configuration = configuration[1:]

    joint_types=[]
    joint_axis=[]
    links=[]
    for i in range(0,len(configuration)-3,4):
        joint_types.append(configuration[i])
        joint_axis.append(configuration[i+1])
        links.append(configuration[i+2]+"."+configuration[i+3])
    # print("joint_types",joint_types)
    # print("joint_axis",joint_axis)
    # print("links",links)

    one_at_a_time(dof,arm_name,simulation_db,joint_types,links)
    print(directory + arm_name_urdf+'.xacro',"first")
    print('/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/tested_arms/' + arm_name_urdf+'.xacro',"second")
    os.rename(directory + arm_name_urdf+'.xacro', '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms/' + arm_name_urdf+'.xacro')
    #os.rename(directory + arm_name_urdf +'.xacro', directory + arm_name_urdf+'.xacro')


    # simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Sucsses", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Mid joint proximity",""])
    # directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
    # # file_number=999
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
    # sim = Simulator(arm_name, dof=4,folder = 'arms',create=False)
    # sim.create_urdf_from_csv(csv_name="all_configs4",num_of_group=None, folder="arms", min_length=1.4)

    ''' Creating 1 URDF
    # '''
    # joint_types=["roll","pris","roll","roll","roll", "pris"]
    # joint_axis= ['z', 'z','y', 'y', 'y','y']
    # links =['0.1', '0.5','0.5', '0.5', '0.3','0.1']
    # arm_name=None
    # sim = simulation(joint_types, joint_axis, links ,arm_name)  
    # sim.generate_urdf()
 
    ''' Calculate links lenghts options amount
    ''' 
    # calc_links_options(min_length=1.4, max_lenght =2,dof=4)
