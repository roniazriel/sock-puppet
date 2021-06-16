#!/usr/bin/env python
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import time
import csv
import pandas as pd


try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

''' code start'''

# def moveResult(data):
# 	print(data)
# 	if(data.result.error_code):
# 		print('goal success')
# 	else:
# 		print('goal failed')
# 	execution_results.append(data.result.error_code)

# rospy.Subscriber("/move_group/result", moveit_msgs.msg.MoveGroupActionResult, moveResult)



plan_results=[]
move_duration=[]
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



def go_home():
	home = move_group.get_current_joint_values()
	home[1] = 0
	home[2] = 0
	home[3] = 0
	home[4] = 0
	home[5] = 0
	home[6] = 0
	move_group.go(home, wait=True)
	move_group.stop()
	print('Home position reached')

# new_pos = move_group.get_current_joint_values()
# new_pos[0] = 1

def go_pose(goal):
	spray_offset_x = 0.35
	vine_offset_x = 0.25
	start = time.time()
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1
	pose_goal.position.x = goal[0] - spray_offset_x- vine_offset_x
	pose_goal.position.y = goal[1]
	pose_goal.position.z = goal[2]

	move_group.set_pose_target(pose_goal)

	plan_result=move_group.go(wait=True)
	plan_results.append(plan_result)
	move_group.stop()
	end = time.time()
	print('Time:',end - start)
	move_duration.append(end - start)
	move_group.clear_pose_targets()

	if(plan_result):
		print("Success - A plan was created")		
		time.sleep(5)
		print('Goal position reached')
	else:
		print("Failed - The goal is out of reach")
		time.sleep(3)

	

go_home()

for goal in grapes_coords:
	go_pose(goal)
	go_home()


print(move_duration)
print(plan_results)

pd.DataFrame(move_duration).to_csv("/home/student/Downloads/move_duration.csv", header=None, index=None)
pd.DataFrame(plan_results).to_csv("/home/student/Downloads/plan_results.csv", header=None, index=None)
pd.DataFrame(grapes_coords).to_csv("/home/student/Downloads/grapes_coords.csv", header=None, index=None)







