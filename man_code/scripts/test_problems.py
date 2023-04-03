import os
import random
import numpy as np
import pandas as pd
import time
import subprocess
from matplotlib import pyplot as plt
from matplotlib import animation
# from tabulate import tabulate
from numpy import mean
from ros import Ros, MoveGroupPythonInterface, UrdfClass
from simulator import Simulator, one_at_a_time
from arm_creation import create_arm, UrdfClass

# _roll_z_0_1_pitch_y_0_1_pitch_z_0_1_pitch_x_0_5_roll_x_0_7_roll_y_0_1.urdf.xacro
arm_name = '_roll_z_0_1_pitch_y_0_1_pitch_z_0_1_pitch_x_0_5_roll_x_0_7_roll_y_0_1'
# joint_types = ['roll','pitch','pitch','pitch','roll','roll']
joint_config_index = 889
link_config_index = 26

print("before try")
arms = ['_roll_z_0_1_pitch_y_0_1_pitch_z_0_5_pitch_x_0_3_roll_x_0_3_pitch_y_0_7','_roll_z_0_1_pitch_y_0_1_pitch_z_0_1_pitch_x_0_5_roll_x_0_7_roll_y_0_1']
indexes = [[847,103],[889,26]]
for i in range(2):
	print(str(indexes[i][0]), str(indexes[i][1]))
	print(arms[i])
	try:
	    # subprocess.Popen(["python", "script.py"] + myList

	    args = [str(indexes[i][0]), str(indexes[i][1]),arms[i]]

	    # a solution to manually ctrl+c in the main code and the process will continue
	    # p = subprocess.call(['gnome-terminal', '--wait','--', '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/simulation_manager_shell.sh']+args)
	    



	    p = subprocess.call(['gnome-terminal', '--', '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/simulation_manager_shell.sh']+args)
	    p.wait(timeout=35)


	    print("I have not waited")
	    time.sleep(30)

	    # os.rename('/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'+ arm_name+'.urdf.xacro', '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/tested_arms/'+ arm_name+'.urdf.xacro')
	    print("here i failed")
	    results_df = pd.read_csv(result_file,index_col=0)
	    results_df["Success"] = results_df["Success"].astype(int)

	    ans = results_df.tail(10).groupby('Arm ID').aggregate({'Success': 'sum', 'Manipulability - mu': 'min'}).reset_index().rename(columns={'Success': 'Reachability', 'Manipulability - mu': 'Min_Manipulability'})

	    Reachability = ans.loc[(ans['Arm ID'] == arm_name)]['Reachability'].values[0]
	    Min_Manipulability = ans.loc[(ans['Arm ID'] == arm_name)]['Min_Manipulability'].values[0]

	except:
	    print("!!!!!!!!Poor Manipulator has been chosen- Execution failed!!!!!!!!")
	    cmd = 'killall -9 gzserver;killall -9 roscore'
	    cmd2 = 'killall -9 roscore'
	    # p2 = subprocess.call(['gnome-terminal', '--disable-factory', '--wait', '--', cmd2])
	    os.system("killall -9 gzserver")
	    os.system("killall -9 roscore")
	    Reachability =0
	    Min_Manipulability = 0

	print("End of code")
