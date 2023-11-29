#!/usr/bin/env python

from ros import Ros, MoveGroupPythonInterface, UrdfClass
from simulator import Simulator, one_in_at_time, multiple_simulations
# from __future__ import print_function
# from arm_creation import UrdfClass, create_arm
# from ros import Ros,MoveGroupPythonInterface 
# import roslaunch
import os
# import subprocess
# import shlex
# from os import listdir, path, mkdir  # , rename
import time
# from six.moves import input
# import sys
# import copy
# import rospy
# import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# import time
# from datetime import datetime
# import csv
# import numpy as np
import pandas as pd
# import random
# from itertools import product
# from std_msgs.msg import String
# from sensor_msgs.msg import JointState
# from moveit_commander.conversions import pose_to_list
import math



""" This file handle all the simulation process:
    1) first create (if create_urdf == True) the dsired urdf files
    2) run over all the urdf files and enter them to the simulator
    3) create csv file (the name of the file is the current date and time) with the results of simulation
 """
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
# import moveit_commander
# import moveit_msgs.msg
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


if __name__ == '__main__':
	simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Sucsses", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Mid joint proximity",""])
	directory = '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
	# file_number=999

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

	        one_in_at_time(arm_name,simulation_db,joint_types,links)

	    os.rename(directory + filename, '/home/ar1/catkin_ws/src/sock-puppet/' + filename)
	    break

