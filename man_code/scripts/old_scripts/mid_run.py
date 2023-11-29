#!/usr/bin/env python
import os
from os import listdir, path, mkdir  # , rename
import pandas as pd

df = pd.read_csv ('/home/arl/roni_ws/src/sock-puppet/results/sim_results4000.csv')
#print(df.ix[1:,"Arm ID"], "df")


directory = '/home/arl/roni_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
new_directory ='/home/arl/roni_ws/src/sock-puppet/man_gazebo/urdf/6dof/processed_arms/'

file_number=1
for filename in os.listdir(directory):
    f = os.path.join(directory, filename)
    # if file_number >=5 :
    # 	break
    # checking if it is a file
    if os.path.isfile(f):
        head,arm_name = os.path.split(f[0:-11])
        # print(head, "head")
        # print(arm_name,"arm_name")
        for name in df.ix[1:,"Arm ID"]:
	        if arm_name == name:
	        	print("in if")
	        	try:
		        	os.rename( directory+ arm_name+".urdf.xacro", new_directory+ arm_name+".urdf.xacro")
		        	print("im here")
	        	except:
		        	print("File already Exists")
	    			print("Removing existing file")
    file_number+=1
