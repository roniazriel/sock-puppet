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
import os

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

	def go_pose(self,goal):
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
			#performance_indicators = self.indices_calc()
			#pd.DataFrame(self.performance_indicators).to_csv(self.path +"/Downloads/performance_indicators.csv", header=None, index=None)
			time.sleep(5) # spraying time
			print('Goal position reached')
		else:
			print("Failed - The goal is out of reach")
			time.sleep(3)# sad

		print(self.move_duration)
		print(self.plan_results)

		pd.DataFrame(self.move_duration).to_csv(self.path +"results/move_duration.csv", header=None, index=None)
		pd.DataFrame(self.plan_results).to_csv(self.path +"results/plan_results.csv", header=None, index=None)
		pd.DataFrame(self.grapes_positions).to_csv(self.path +"results/grapes_coords.csv", header=None, index=None)


	
	def start_spray(self):
		print('start_spray')
		# self.home= self.move_group.get_current_pose()
		for goal in self.grapes_positions:
			self.go_pose(goal)
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


if __name__ == '__main__':
	spray= Spray()
	spray.start_spray()





