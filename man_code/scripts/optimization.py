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

""" This is the framwork of the optimization process -  PSO implementation
    
    A particle in the swarm :

    
    His position is a robotic manipualator that is compossed by 
    
    The Velocity is the jump it will take to the next manipulator. 

    Neighborhood is considered as 

"""
class RoboticPosition:
    def __init__(self, joint_types,joint_axis,links,arm_name,joint_index,link_index ):
    # """ Position object, which refers to robotic manipulator. 
    #     the position includes the manipulator joint configs and link config """

        # An index of positions in joint and link configuration 
        self.joint_config_index = joint_index
        self.link_config_index = link_index

        # Robotic position data
        self.joint_types = joint_types
        self.joint_axis = joint_axis
        self.links = links
        self.arm_name = arm_name


class Particle:
    def __init__(self,Pid, joint_config_indexs,link_config_indexs,seed):
        ''' Create Particle
            Pid: particle id
            :param joint_config_index: joint configuration index
            :param link_config_index: link configuration index

            self.position - current position
            self.velocity - current velocity
            self.pbest_position - personal best position
            self.pbest_fitness - personal best fitness

        '''
        # initialize seed for repetitivity
        self.id = Pid
        self.seed = seed

        # csv files with the feasible solutions
        self.joint_indexs = joint_config_indexs
        self.link_indexs = link_config_indexs

        # initialize joint config velocity of the particle with 0.0 value
        self.joint_config_velocity = 0

        # initialize link config velocity of the particle with 0.0 value
        self.link_config_velocity = 0

        # initialize an indicator for switching between joint configurations 
        self.jump_joint_config = 1

        # initialize position of the particle with randomly
        self._update_position(initiate=True) # initialize also best particle position to initial position
        self._fitness_function(pbest=True) # initialize also best particle fitness of the particle

        # initialize best particle position to initial position
        # self._update_position(initiate=True,pbest=True)
        print("pbest pos", self.pbest_position.joint_config_index, self.pbest_position.link_config_index)

        # initialize best particle fitness of the particle
        # self._fitness_function(pbest=True)
        print("pbest fit",self.pbest_fitness)

    
    def _fitness_function(self,pbest=False, result_file='/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/optimization_results_tests.csv'):
        """ Returns objective function value of a state """
        
        if pbest:
            result_file = result_file
            arm_name = self.pbest_position.arm_name
            joint_types = self.pbest_position.joint_types
            links = self.pbest_position.links
            args = [str(self.pbest_position.joint_config_index), str(self.pbest_position.link_config_index),arm_name]
        else:
            result_file = result_file
            arm_name = self.position.arm_name
            joint_types = self.position.joint_types
            links = self.position.links
            args = [str(self.position.joint_config_index), str(self.position.link_config_index),arm_name]

        simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Success", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Manipulability - roni","Mid joint proximity",
                                            "Max Mid joint proximity","Sum Mid joint proximity","Sum Mid joint proximity- all joints"])
        
        
        # subprocess.Popen(["python", "script.py"] + myList)
        subprocess.check_call(['/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/simulation_manager_shell.sh']+args)

        # one_at_a_time(6,arm_name,simulation_db,joint_types,links,result_file)
        # time.sleep(2.4)
        os.rename('/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'+ arm_name+'.urdf.xacro', '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/tested_arms/'+ arm_name+'.urdf.xacro')

        results_df = pd.read_csv(result_file,index_col=0)
        results_df["Success"] = results_df["Success"].astype(int)

        ans = results_df.groupby('Arm ID').aggregate({'Success': 'sum', 'Manipulability - mu': 'min'}).reset_index().rename(columns={'Success': 'Reachability', 'Manipulability - mu': 'Min_Manipulability'})

        Reachability = ans.loc[(ans['Arm ID'] == arm_name)]['Reachability'].values[0]
        Min_Manipulability = ans.loc[(ans['Arm ID'] == arm_name)]['Min_Manipulability'].values[0]

        if Reachability < 10:
            objective = 0
        else:
            objective = Min_Manipulability

        if pbest:
            self.pbest_fitness = objective
            self.position_fitness = objective
        else:
            self.position_fitness = objective


    def _update_velocity(self, gbest_position,jump_threshold):
        """
        pbest: [personal best Joint configuration, personal best Link configuration]
        gbest: [global best Joint configuration, global best Link configuration]
        """
        # Randomly generate r1, r2 and inertia weight from normal distribution
        r1 = random.random()
        r2 = random.random()

        
        # hyper parameters
        w = 0.729    # inertia
        c1 = 1.49445 # cognitive (particle)
        c2 = 1.49445 # social (swarm)

        # Calculate new velocity
        self.jump_joint_config +=1

        if self.jump_joint_config%jump_threshold==0:            
            new_joint_velocity = w * self.joint_config_velocity + c1 * r1 * (self.pbest_position.joint_config_index - self.position.joint_config_index) + c2 * r2 * (gbest_position.joint_config_index - self.position.joint_config_index)
            self.joint_config_velocity = int(new_joint_velocity)

            # if velocity[k] is not in [minx, max]
            # then clip it
            if self.joint_config_velocity + self.position.joint_config_index < 1:
                self.position.joint_config_index = 1
            elif self.joint_config_velocity + self.position.joint_config_index > 6836:
                self.position.joint_config_index = 6836
            print("new joint velocity: ", new_joint_velocity)

        else:
            new_link_velocity = w * self.link_config_velocity + c1 * r1 * (self.pbest_position.link_config_index - self.position.link_config_index) + c2 * r2 * (gbest_position.link_config_index - self.position.link_config_index)
            self.link_config_velocity = int(new_link_velocity)
            print("new link velocity: ", new_link_velocity )


    def _update_position(self, initiate, pbest=False):

        if initiate:
            joint_index = random.randint(1,6836)
            link_index = random.randint(1,456)
        else:
            joint_index = self.position.joint_config_index
            link_index = self.position.link_config_index

        # Update indexs by velocity (For initiating pbest, velocity will be 0)
        if self.link_config_velocity + link_index < 1:
            link_index = 1
        elif self.link_config_velocity + link_index > 456:
            link_index = 456
        else:
            link_index = link_index + self.link_config_velocity

        if self.joint_config_velocity + joint_index < 1:
            joint_index = 1
        elif self.joint_config_velocity + joint_index > 6836:
            joint_index = 6836
        else:
            joint_index = joint_index + self.joint_config_velocity


        print("new joint index: ", joint_index)
        print("new link index: ", link_index)

        # Get new particle info
        joints = self.joint_indexs.loc[self.joint_indexs["Index"]==joint_index]
        links = self.link_indexs.loc[self.link_indexs["Index"]==link_index]

        joint_types=[joints['Joint'+str(i+1)+'_Type'].values[0] for i in range(6)]
        joint_axis=[joints['Joint'+str(i+1)+'_Axis'].values[0] for i in range(6)]
        links_lengths = [str(links['Link'+str(i+1)+'_Length'].values[0]) for i in range(6)]

        print("Joints types: ", joint_types)
        print("Joints axis: ", joint_axis)
        print("Links lengths: ", links_lengths)

        # Create URDF - the particle
        arm_name = create_arm(joint_types,joint_axis,links_lengths)
        print("Arm Name: ",arm_name)

        # Update particle
        pos = RoboticPosition(joint_types,joint_axis,links_lengths,arm_name, joint_index,link_index)

        if initiate:
            self.pbest_position = pos
            self.position = pos
        else:
            self.position = pos


    def write_track_to_csv(self,db,i):
        db = db.append({"Iteration_number": i,
            "ParticleID": self.id,
            "Link_Velocity": self.link_config_velocity,
            "Joint_Velocity":self.joint_config_velocity,
            "Current_Link_Index": self.position.link_config_index, 
            "Current_Joint_Index": self.position.joint_config_index,
            "Current_Joints_Types": self.position.joint_types,
            "Current_Joints_Axis": self.position.joint_axis,
            "Current_Links_Lengths": self.position.links,
            "Current_Fitness":self.position_fitness,
            "pbest_Link_Index": self.pbest_position.link_config_index, 
            "pbest_Joint_Index": self.pbest_position.joint_config_index,
            "pbest_Joints_Types": self.pbest_position.joint_types,
            "pbest_Joints_Axis": self.pbest_position.joint_axis,
            "pbest_Links_Lengths":self.pbest_position.links,
            "pbest_Fitness": self.pbest_fitness},ignore_index=True)
        return db

# particle swarm optimization function
def pso(joint_config_indexs,link_config_indexs, generation_num, population_size, jump_threshold, track_file):
        
    # create n random particles
    swarm = [Particle(i,joint_config_indexs,link_config_indexs, i) for i in range(population_size)]
 
    # initiate swarm's best_position and best_fitness
    gbest_position = swarm[0].position
    gbest_fitness = swarm[0].position_fitness

    # compute the best particle of swarm and it's fitness
    for i in range(population_size): # check each particle
        if swarm[i].position_fitness > gbest_fitness:
          gbest_fitness = swarm[i].position_fitness
          gbest_position = swarm[i].position

    # Particle initialization tracking    
        pso_track = pd.DataFrame(columns=["Iteration_number","ParticleID", "Link_Velocity","Joint_Velocity","Current_Link_Index", 
        "Current_Joint_Index", "Current_Joints_Types","Current_Joints_Axis","Current_Links_Lengths","Current_Fitness", "pbest_Link_Index", 
        "pbest_Joint_Index", "pbest_Joints_Types","pbest_Joints_Axis","pbest_Links_Lengths","pbest_Fitness"])

        initialization_track = swarm[i].write_track_to_csv(pso_track, "Initialization")

        if(os.path.isfile(track_file)):
            pd.DataFrame(initialization_track).to_csv(track_file, mode='a', index=False, header=False)
        else:
            pd.DataFrame(initialization_track).to_csv(track_file, index=False)

    # main loop of pso
    Iter = 0

    while Iter < generation_num:
        
        # after every 10 iterations
        # print iteration number and best fitness value so far
        if Iter % 10 == 0 and Iter > 1:
          print("Iter = " + str(Iter) + " best fitness = %.3f" % gbest_fitness + " best position = %.3f" % gbest_position)

        for k in range(population_size): # process each particle

            swarm[k]._update_velocity(gbest_position,jump_threshold)
          # compute new position using new velocity
            swarm[k]._update_position(initiate=False)

          # compute fitness of new position
            swarm[k]._fitness_function()

          # is new position a new best for the particle?
            if swarm[k].position_fitness > swarm[k].pbest_fitness:
                swarm[k].pbest_fitness = swarm[k].position_fitness
                swarm[k].pbest_position = swarm[k].position

          # is new position a new best overall?
            if swarm[k].position_fitness > gbest_fitness:
                gbest_fitness = swarm[k].position_fitness
                gbest_position = swarm[k].position

        # Keeping track of the particles throughout the algorithm run
            iteration_track = swarm[k].write_track_to_csv(pso_track, Iter)
            print(iteration_track)
            if(os.path.isfile(track_file)):
                pd.DataFrame(iteration_track).to_csv(track_file, mode='a', index=False, header=False)
            else:
                pd.DataFrame(iteration_track).to_csv(track_file, index=False)

        Iter += 1
    
    return gbest_position, gbest_fitness


if __name__ == '__main__':

    joint_config_index = pd.read_csv("Sorted_Joint_Configs_Indexs.csv")
    link_config_index = pd.read_csv("Sorted_Link_Configs_Indexs.csv")

    track_file = '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/optimization_track.csv'
    gbest_position, gbest_fitness = pso(joint_config_indexs=joint_config_index,link_config_indexs=link_config_index, generation_num=2, population_size=2, jump_threshold=10, track_file=track_file)
    print("Final Result:  " + " Best fitness = %.3f" % gbest_fitness + " , Best Arm = " +str(gbest_position.arm_name) + " , Best Joint index = " + str(gbest_position.joint_config_index)+ " , Best Link Index = " + str(gbest_position.link_config_index))
