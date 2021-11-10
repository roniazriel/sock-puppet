
""" This file handle all the simulation process:
    1) first create (if create_urdf == True) the dsired urdf files
    2) run over all the urdf files and enter them to the simulator
    3) create csv file (the name of the file is the current date and time) with the results of simulation
 """
from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
from datetime import datetime
from os import environ, listdir, path, mkdir  # , rename
import numpy as np
from itertools import product
from time import sleep
from rospy import init_node
from rosnode import get_node_names
import getpass
import sys
import json
import rospy


class Simulator(object):

    def __init__(self, dof=6, folder='combined', create=False, arms=None,  wait1=2.3, wait2=2.7, link_max=0.41):
        # if arms is None:
        #   arms = []
        self.dof = dof
        self.folder = folder
        self.ros = Ros()  # for work with Ros
        self.arm_control = 0
        self.arms = []
        self.wait1 = wait1
        self.wait2 = wait2
        self.json_data = []
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv(str(self.dof) + "dof_configs",  link_max)
        else:
            if not arms:
                self.arms_exist()
            else:
                self.arms = arms
        # desired positions and orientaions of the EE in world frame
        z = 3  # height from ground
        # self.poses = [[0.5, 0, z + 0.9], [0.2, 0, z + 0.9], [0.2, 0.0, z + 0.65], [0.2, 0, z + 0.4]]
        # self.oriens = [[-3.14, 0, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.25, 0]]
        grapes_coords =  [[1.2,2.9,1.5],
             [1.14,2.08,1.57],
             [1.51,1.42,1.85],
             [1.26,0.08,2.06],

             [1.05,-0.15,1.3],
             [1.43,-1.53,1.47],
             [1.18,-1.42,1.6],
             [1.1,-3.15,1.5],
             [1.16,-4.2,1.7],
             [1.4,-4.5,1.8]]
        self.poses = [[0.5, 0, z + 0.9], [0.55, 0, z + 0.9], [0.55, 0, z + 0.95], [0.5, 0, z + 0.95], [0.45, 0, z + 0.95],
                      [0.45, 0, z + 0.9], [0.45, 0, z + 0.85], [0.5, 0, z + 0.85], [0.55, 0, z + 0.85],
                      [0.2, 0, z + 0.9], [0.25, 0, z + 0.9], [0.25, 0, z + 0.95], [0.2, 0, z + 0.95], [0.15, 0, z + 0.95],
                      [0.15, 0, z + 0.9], [0.15, 0, z + 0.85], [0.2, 0, z + 0.85], [0.25, 0, z + 0.85],
                      [0.2, 0.0, z + 0.65], [0.25, 0.0, z + 0.65], [0.25, 0.0, z + 0.7], [0.2, 0.0, z + 0.7], [0.15, 0.0, z + 0.7],
                      [0.15, 0.0, z + 0.65], [0.15, 0.0, z + 0.6], [0.2, 0.0, z + 0.6], [0.25, 0.0, z + 0.6],
                      [0.2, 0, z + 0.4],  [0.25, 0, z + 0.4],  [0.25, 0, z + 0.45],  [0.2, 0, z + 0.45], [0.15, 0, z + 0.45],
                      [0.15, 0, z + 0.4],  [0.15, 0, z + 0.35],  [0.2, 0, z + 0.35],  [0.25, 0, z + 0.35]]
        self.oriens = [[0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0],
                       [0, -3.14, 0], [0, -3.14, 0], [0, -3.14, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0],
                       [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0],
                       [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.5, 0],
                       [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0],
                       [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0],
                       [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0],
                       [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0], [0, 3.1459*0.36, 0]]
        self.save_name = 'results_file' + datetime.now().strftime("%d_%m_") + str(dof) + "dof_" \
                        + str(len(self.poses)) + "d_"
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof"]
        sleep(0.1)
        self.main = self.ros.start_launch("main", "man_gazebo", main_launch_arg)  # main launch file
        # set the obstacles and initiliaze the manipulator
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        # add floor and plant to the planning model
        sleep(0.12)
        self.manipulator_move.add_obstacles(height=z + 0.75, radius=0.1, pose=[0.5, 0])
        pos = self.manipulator_move.get_current_position()
        orien = self.manipulator_move.get_current_orientain()
        self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], [orien[0], orien[1], orien[2]])
        self.replace_model(-1)  # set the first arm

    @staticmethod
    def save_json(name="data_file", data=None):
        with open(name + ".json", "a") as write_file:
            json.dump(data, write_file, indent=2)

    @staticmethod
    def load_json(name="data_file"):
        with open(name + ".json", "r") as read_file:
            return json.load(read_file)

    @staticmethod
    def create_arm(interface_joints, joint_parent_axis, links, folder):
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
            file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + links[i].replace(".", "_")
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
        arm = UrdfClass(links, joints, joint_axis, rpy)
        return {"arm": arm, "name": file_name, "folder": folder}

    def set_links_length(self, min_length=1, link_min=0.1, link_interval=0.3, link_max=0.71):
        """
        set all the possible links lengths in the defind interval
        :param min_length: the minimum length of all the links
        :param link_min: minimum length of a link
        :param link_interval: interval between joints lengths
        :param link_max: maximun length of a link
        :return: links: all the lengths combinations in the minimum length - list of strings
        """

        links = []
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        links_length = [[0.1] + list(tup) for tup in
                        list(product(lengths_2_check, repeat=(self.dof - 1)))]
        for link in links_length:
            if sum(link) > min_length:
                links.append([str(x) for x in link])
        return links

    def create_urdf_from_csv(self, csv_name="manips", link_max=0.41):
        # read from csv file with all the possible configuration for manipulators
        configs = HandleCSV().read_data(csv_name)
        # Create the urdf files
        data = []
        base_path = environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/"
        self.create_folder(base_path + str(self.dof) + "dof/" + self.folder)
        links = self.set_links_length(link_max=link_max)
        index = 0
        folder_num = 0
        folder = self.folder
        for config in configs:
            for arm in config:
                for link in links:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"].replace(" ", ""), folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        # self.save_json("created arms", data)

    def arms_exist(self):
        path = environ['HOME'] + "/catkin_ws/src/sock-puppet/man_gazebo/urdf/" + str(self.dof) \
               + "dof/" + self.folder
        for fil in listdir(path):
            fol = self.folder.split("/")
            data = self.get_urdf(name=fil.replace(".urdf.xacro", ""))
            self.arms.append(self.create_arm(data[0], data[1], data[2], fol[0]))
            # self.arms.append({"name": fil.replace(".urdf.xacro", ""), "folder": fol[0]})

    @staticmethod
    def get_urdf(name=""):
        # This function implement joints tpye and axis in the relevant variables  
        # First joint is always "roll_z_0.1" 
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

    @staticmethod
    def create_folder(name):
        if not path.exists(name):
            mkdir(name)
        return name

    def assign_data(self, data, arm, k):
        """
        Calculate the manipulaot indices(Manipulability, Local Conditioning Index, Joint Mid-Range Proximity)
        if the manipulator succed and the time that take
        :param data: array of the result of the configuration about each detection point
        :param arm: which configuration
        :return: array of the results
        """
        data_res = []
        jacobian = []
        curr_pos = []
        mu = []   # Manipulability index
        z = []    # Joint Mid-Range Proximity

        arm_number = str(arm + 1 + k)
        for j in data:
            data_res.append(j[0])
            if j[0]:
                mu.append(j[2][0])
                z.append(j[2][1].min())
                jacobian.append(j[2][2].tolist())
                curr_pos.append(j[2][3].tolist())
            else:
                mu.append(-1)
                z.append(-1)
                jacobian.append(-1)
                curr_pos.append(-1)
        self.json_data.append({self.arms[arm]["name"]: [jacobian, curr_pos]})
        suc_res = "False"
        mu_min = -1
        z_max = -1
        data_time = [-1, -1, -1, -1]
        avg_time = -1
        if True:  # data_res.count(True) >= 3 and data_res[3] and (data_res[0] or data_res[1]):
            # if the arm arrived to 3 or more point and get to the lower point or one of the two
            # top points --> it success and calc indices
            suc_res = "True"
            data_time = [d[1] for d in data]
            avg_time = np.mean(data_time).round(2)
            mu = np.asarray(mu)
            z = np.asarray(z)
            # choose only the min values because those are the "worst grade"
            try:
                mu_min = mu[mu >= 0.0].min()
            except:
                self.save_json("mu_err", mu.tolist())
                mu_min = -16
            try:
                z_max = z[z >= 0.0].max()
            except:
                self.save_json("z_err", z.tolist())
                z_max = 16
        return [arm_number, datetime.now().strftime("%d/%m/%y, %H:%M"), self.arms[arm]["name"], data_res,
                str(data_time), suc_res,  str(avg_time), str(mu_min), str(z_max)]

    def replace_model(self, arm):
        """
        replace configuration in the simulation
        :param arm: the new configuration to simulate
        """
        fil = "man:=" + self.arms[arm + 1]["folder"] + "/" + self.arms[arm + 1]["name"] + \
              " dof:=" + str(self.dof) + "dof"
        if self.arm_control != 0:
            self.ros.stop_launch(self.arm_control)  # this launch file must be stopped, otherwise it wont work
        else:
            self.ros.ter_command("rosnode kill /robot_state_publisher")
        replace_command = "roslaunch man_gazebo replace_model.launch " + fil
        self.ros.ter_command(replace_command)
        sleep(self.wait1)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        sleep(self.wait2)

    def run_simulation(self,  k=0, len_arm=1638):
        save_name = self.save_name  # file to save the results
        all_data = []
        for arm in range(0, len(self.arms)):
            print(self.arms[arm]["name"] + " " + str(arm + 1 + k) + " of " + str(len_arm) + " arms")
            data = []
            joints = self.arms[arm]["arm"].joint_data
            links = self.arms[arm]["arm"].links
            for p in range(len(self.poses)):  # send the manipulator to the selected points
                # inserting the data into array
                data.append(self.manipulator_move.go_to_pose_goal(self.poses[p], self.oriens[p], joints, links))
            # calculate relavent data from data array
            all_data.append(self.assign_data(data, arm, k))
            if arm == len(self.arms) - 1:
                break
            self.replace_model(arm)
        # save the remaining data and close all the launch files
        self.save_json(save_name, self.json_data)
        HandleCSV().save_data(all_data, save_name)
        self.ros.stop_launch(self.arm_control)
        self.ros.stop_launch(self.main)


def simulate(start_arm=0, from_opt=True):
    # default values
    dofe = 6  # number degrees of freedom of the manipulator
    link_max = 0.71  # max link length to check
    nums = 25  # how many arms to send to simulator each time
    wait1_replace = 3
    wait2_replace = 3
    start_arm = start_arm / nums
    create_urdf = False
    if not from_opt:
        # set parametrs from terminal
        args = sys.argv
        if len(args) > 1:
            dofe = int(args[1])
            if len(args) > 2:
                start_arm = int(args[2]) / nums
                create_urdf = False
                if len(args) > 3:
                    link_max = float(args[3]) + 0.1
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
    
    # folder to save the file to
    foldere = "combined"
    sim = Simulator(dof=dofe, folder=foldere, create=create_urdf, wait1=wait1_replace, wait2=wait2_replace, link_max=link_max)
    if start_arm > 0:
        ros.stop_launch(sim.arm_control)
        ros.stop_launch(sim.main)
    arms = sim.arms
    if len(arms) < nums:
        nums = len(arms)
    nodes = get_node_names()
    for node in nodes:
        if "/moveit_python_wrappers" in node:
            # moveit_arg = "call --wait " + node + """set_logger_level &quot; {logger: 'ros', level: 'Warn'} &quot;" """
            command = "roslaunch man_gazebo logging_level.launch moveit_log:=" + node
            ros.ter_command(command)
    for t in range(start_arm, int(np.ceil(1.0*len(arms) / nums))):
        if t == len(arms) / nums:
            sim = Simulator(dof=dofe, folder=foldere, create=False, arms=arms[t * nums:], wait1=wait1_replace, wait2=wait2_replace)
            sim.run_simulation(nums*t, len(arms))
        elif t != 0:
            sim = Simulator(dof=dofe, folder=foldere, create=False, arms=arms[t * nums:(t + 1) * nums],wait1=wait1_replace, wait2=wait2_replace)
            sim.run_simulation(nums*t, len(arms))
        else:  # first run
            sim.arms = arms[:nums]
            sim.run_simulation(nums*t, len(arms))
    sleep(2)
    ros.ros_core_stop()
    with open("finish.txt", "w+") as f:
        f.write("finish")
        f.close()
    sleep(2)
    roscore = ros.checkroscorerun()
    if roscore:
        ros.ter_command("kill -9 " + str(roscore))


if __name__ == '__main__':
    simulate(from_opt=False)