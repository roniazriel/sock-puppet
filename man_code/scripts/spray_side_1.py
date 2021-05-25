#!/usr/bin/env python
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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


coords= [[0.4,2,1.53],[0.4,4,1.53]]

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
	print('at home')

# new_pos = move_group.get_current_joint_values()
# new_pos[0] = 1

def go_pose(goal):
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1
	pose_goal.position.x = goal[0]
	pose_goal.position.y = goal[1]
	pose_goal.position.z = goal[2]

	move_group.set_pose_target(pose_goal)

	move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	print('at pos')

go_home()

for goal in coords:
	go_pose(goal)
	time.sleep(5)
	go_home()








