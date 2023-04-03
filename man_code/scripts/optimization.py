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
import csv
from datetime import datetime

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
    def __init__(self,Pid, joint_config_indexs,link_config_indexs,seed,sim_res_file):
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
        self.max_joint_config_velocity = 6836 # max velocity = max x
        random.seed(self.seed)
        self.joint_config_velocity = random.randint(2,10)
        print("initiate joint vel",self.joint_config_velocity)

        # initialize link config velocity of the particle with 0.0 value
        self.max_link_config_velocity = 456 # max velocity = max x
        random.seed(self.seed*2)
        self.link_config_velocity = random.randint(2,10)
        print("initiate link vel",self.link_config_velocity)

        # initialize an indicator for switching between joint configurations 
        self.jump_joint_config = 1

        # initialize position of the particle with randomly
        self._update_position(initiate=True) # initialize also best particle position to initial position
        self._fitness_function(pbest=True,result_file=sim_res_file) # initialize also best particle fitness of the particle

        # initialize best particle position to initial position
        # self._update_position(initiate=True,pbest=True)
        print("pbest pos", self.pbest_position.joint_config_index, self.pbest_position.link_config_index)

        # initialize best particle fitness of the particle
        print("pbest fit",self.pbest_fitness)

    
    def _fitness_function(self,pbest=False, result_file='/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/optimization_results_tests1.csv'):
        """ Returns objective function value of a state """
        
        if pbest:
            arm_name = self.pbest_position.arm_name
            joint_types = self.pbest_position.joint_types
            links = self.pbest_position.links
            args = [str(self.pbest_position.joint_config_index), str(self.pbest_position.link_config_index),arm_name,result_file]
        else:
            
            arm_name = self.position.arm_name
            joint_types = self.position.joint_types
            links = self.position.links
            args = [str(self.position.joint_config_index), str(self.position.link_config_index),arm_name,result_file]

        simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Success", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Manipulability - roni","Mid joint proximity",
                                            "Max Mid joint proximity","Sum Mid joint proximity","Sum Mid joint proximity- all joints"])
        
        try:
            # subprocess.Popen(["python", "script.py"] + myList)
            subprocess.call(['gnome-terminal', '--wait','--', '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/simulation_manager_shell.sh']+args)
            # print("I have not waited")
            # time.sleep(30)

            os.rename('/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'+ arm_name+'.urdf.xacro', '/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/tested_arms/'+ arm_name+'.urdf.xacro')

            results_df = pd.read_csv(result_file,index_col=0)
            results_df["Success"] = results_df["Success"].astype(int)

            ans = results_df.tail(10).groupby('Arm ID').aggregate({'Success': 'sum', 'Manipulability - mu': 'min'}).reset_index().rename(columns={'Success': 'Reachability', 'Manipulability - mu': 'Min_Manipulability'})

            Reachability = ans.loc[(ans['Arm ID'] == arm_name)]['Reachability'].values[0]
            Min_Manipulability = ans.loc[(ans['Arm ID'] == arm_name)]['Min_Manipulability'].values[0]
        except:
            print("!!!!!!!!Poor Manipulator has been chosen!!!!!!!!")
            os.system("killall -9 gzserver")
            os.system("killall -9 roscore")
            Reachability = -1
            Min_Manipulability = -1
        

        if pd.isna(Min_Manipulability):
            Min_Manipulability = 0
        
        objective = [Reachability,Min_Manipulability]

        if pbest:
            self.pbest_fitness = objective
            self.position_fitness = objective
        else:
            self.position_fitness = objective


    def _update_velocity(self, gbest_position,w,jump_threshold):
        """
        pbest: [personal best Joint configuration, personal best Link configuration]
        gbest: [global best Joint configuration, global best Link configuration]
        """
        # Randomly generate r1, r2 and inertia weight from normal distribution
        random.seed(self.seed)
        r1 = random.random()
        random.seed(self.seed+1)
        r2 = random.random()

        
        # hyper parameters
        # w = 0.729    # inertia [0.9,1.2] and lineary decreasing ?
        c1 = 1.49445 # cognitive (particle)
        c2 = 1.49445 # social (swarm)

        # c1 = 2.8 # cognitive (particle)
        # c2 = 1.3 # social (swarm)

        # Calculate new velocity
        self.jump_joint_config +=1

        if self.jump_joint_config%jump_threshold==0:           
            new_joint_velocity = w * self.joint_config_velocity + c1 * r1 * (self.pbest_position.joint_config_index - self.position.joint_config_index) + c2 * r2 * (gbest_position.joint_config_index - self.position.joint_config_index)
            
            if abs(new_joint_velocity) <= self.max_joint_config_velocity:
                self.joint_config_velocity = int(new_joint_velocity)
            else:
                self.joint_config_velocity = self.max_joint_config_velocity
            print("new joint velocity: ", new_joint_velocity)

        else:
            new_link_velocity = w * self.link_config_velocity + c1 * r1 * (self.pbest_position.link_config_index - self.position.link_config_index) + c2 * r2 * (gbest_position.link_config_index - self.position.link_config_index)
            if abs(new_link_velocity) <= self.max_link_config_velocity:
                self.link_config_velocity = int(new_link_velocity)
            else:
                self.link_config_velocity = self.max_link_config_velocity
            print("new link velocity: ", new_link_velocity )


    @staticmethod
    def LDIW(max_iter, current_iter):
        """
        Linear Decreasing Inertia Weight
        """
        wi = 0.9
        wf = 0.4
        inertia = (wi-wf)*(((float(max_iter- (current_iter+1))/max_iter))/max_iter) +wf
        print("NEW INERTIA: ", inertia)
        return inertia


    @staticmethod
    def CLDIW(max_iter, current_iter,zk):
        """
        Chaotic Linear Decreasing Inertia Weight
        """
        wi = 0.9
        wf = 0.4
        new_zk= 4*zk*(1-zk)
        inertia = (wi-wf)*(((float(max_iter- (current_iter+1))/max_iter))/max_iter) + wf*new_zk
        return inertia,new_zk

    def _update_position(self, initiate, jump_threshold=5,pbest=False):

        if initiate:
            random.seed(self.seed)
            joint_index = random.randint(1,6836)
            link_index = random.randint(1,456)
            print("joint_index",joint_index)
            print("link_index",link_index)
        else:
            joint_index = self.position.joint_config_index
            link_index = self.position.link_config_index

        # If the threshold value was reached - change the position of the joints index
        if self.jump_joint_config%jump_threshold==0: 
            # position has to be between [min joint_index , max joint_index ]
            if self.joint_config_velocity + joint_index < 1:
                joint_index = 1
            elif self.joint_config_velocity + joint_index > 6836:
                joint_index = 6836
            else:
                joint_index = joint_index + self.joint_config_velocity
        else:
            # Otherwise change position of the links index
            # position has to be between [min link_index , max link_index ]
            if self.link_config_velocity + link_index < 1:
                link_index = 1
            elif self.link_config_velocity + link_index > 456:
                link_index = 456
            else:
                link_index = link_index + self.link_config_velocity


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


    def write_track_to_csv(self,db,i,gbest_position,gbest_fitness):
        db = db.append({"Iteration_number": i,
            "ParticleID": self.id,
            "Particle_seed":self.seed,
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
            "pbest_Fitness": self.pbest_fitness,
            "gbest_Link_Index": gbest_position.link_config_index, 
            "gbest_Joint_Index": gbest_position.joint_config_index,
            "gbest_Joints_Types": gbest_position.joint_types,
            "gbest_Joints_Axis": gbest_position.joint_axis,
            "gbest_Links_Lengths":gbest_position.links,
            "gbest_Fitness": gbest_fitness},ignore_index=True)
        return db

# particle swarm optimization function
def show_particles(swarm,iter_num , show, colors,fig_title):
    particles_joints_indexes = [swarm[i].position.joint_config_index for i in range(len(swarm))]
    particles_links_indexes = [swarm[i].position.link_config_index for i in range(len(swarm))]
    names = [i for i in range(len(swarm))]

    fig, ax = plt.subplots()
    ax.set_xlim([1, 500])
    ax.set_ylim([1, 7000])
    ax.set_title("Iteration Number: " +str(iter_num))
    ax.set_xlabel("Link Configuration Index")
    ax.set_ylabel("Joint Configuration Index")
    ax.scatter(particles_links_indexes, particles_joints_indexes, c=colors)

    for i, txt in enumerate(names):
        ax.annotate(txt, (particles_links_indexes[i], particles_joints_indexes[i]))

    if show:
        plt.show()
    else:
        plt.savefig(fig_title+"IterationPlot_"+str(iter_num)+".png")

def pso(joint_config_indexs,link_config_indexs, generation_num, population_size, w_type,jump_threshold, track_file, sim_res_file, seeds):
        
    # create n random particles
    swarm = [Particle(i,joint_config_indexs,link_config_indexs, seeds[i], sim_res_file) for i in range(population_size)]
 
    # initiate swarm's best_position and best_fitness
    gbest_position = swarm[0].position
    gbest_fitness = swarm[0].position_fitness

    # compute the best particle of swarm and it's fitness
    for i in range(population_size): # check each particle
        if swarm[i].position_fitness[0] > gbest_fitness[0]: # if the robot reached more clusters, its position is better
              gbest_fitness = swarm[i].position_fitness
              gbest_position = swarm[i].position
        elif swarm[i].position_fitness[0] == gbest_fitness[0]:
            if swarm[i].position_fitness[1] > gbest_fitness[1]:
                gbest_fitness = swarm[i].position_fitness
                gbest_position = swarm[i].position

    # Particle initialization tracking    
        pso_track = pd.DataFrame(columns=["Iteration_number","ParticleID", "Particle_seed","Link_Velocity","Joint_Velocity","Current_Link_Index", 
        "Current_Joint_Index", "Current_Joints_Types","Current_Joints_Axis","Current_Links_Lengths","Current_Fitness", "pbest_Link_Index", 
        "pbest_Joint_Index", "pbest_Joints_Types","pbest_Joints_Axis","pbest_Links_Lengths","pbest_Fitness","gbest_Link_Index", "gbest_Joint_Index",
        "gbest_Joints_Types","gbest_Joints_Axis","gbest_Links_Lengths","gbest_Fitness"])
        pd.DataFrame(pso_track).to_csv(track_file, mode='a', index=False)

        initialization_track = swarm[i].write_track_to_csv(pso_track, "Initialization",gbest_position=gbest_position,gbest_fitness=gbest_fitness)

        if(os.path.isfile(track_file)):
            pd.DataFrame(initialization_track).to_csv(track_file, mode='a', index=False, header=False)
        else:
            pd.DataFrame(initialization_track).to_csv(track_file, index=False,header=False)

    colors = np.random.random(len(swarm))
    show_particles(swarm,0,False,colors,"")

    # main loop of pso
    Iter = 0

    # generate random chaotic number
    zk = random.random()
    # initialization rules
    if ((zk==0.0) or (zk==0.25) or (zk==0.5) or (zk == 0.75) or (zk==1.0)) :
        zk = random.random()

    while Iter < generation_num:
        
        # after every 10 iterations
        # print iteration number and best fitness value so far
        # if Iter % 10 == 0 and Iter > 1:
        #   print("Iter = " + str(Iter) + " best fitness = %.3f" % gbest_fitness + " best position = %.3f" % gbest_position)

        # calculate inertia:
        if w_type == 'LDWI':
            w = Particle.LDIW(max_iter=generation_num, current_iter=Iter)
        elif w_type == 'CLDIW':
            w,zk= Particle.CLDIW(max_iter=generation_num, current_iter=Iter, zk=zk)
        else:
            w=0.729

        print("INERTIA WEIGHT VALUE: ",w)

        for k in range(population_size): # process each particle

            swarm[k]._update_velocity(gbest_position,w,jump_threshold)
          # compute new position using new velocity
            swarm[k]._update_position(initiate=False,jump_threshold=jump_threshold)

          # compute fitness of new position
            swarm[k]._fitness_function(result_file=sim_res_file)

          # is new position a new best for the particle?
            if swarm[k].position_fitness[0] > swarm[k].pbest_fitness[0]:
                swarm[k].pbest_fitness = swarm[k].position_fitness
                swarm[k].pbest_position = swarm[k].position

            elif swarm[k].position_fitness[0] == swarm[k].pbest_fitness[0]:
                if swarm[k].position_fitness[1] > swarm[k].pbest_fitness[1]:
                    swarm[k].pbest_fitness = swarm[k].position_fitness
                    swarm[k].pbest_position = swarm[k].position

          # is new position a new best overall?
            if swarm[k].position_fitness[0] > gbest_fitness[0]:
                gbest_fitness = swarm[k].position_fitness
                gbest_position = swarm[k].position

            elif swarm[k].position_fitness[0] == gbest_fitness[0]:
                if swarm[k].position_fitness[1] > gbest_fitness[1]:
                    gbest_fitness = swarm[k].position_fitness
                    gbest_position = swarm[k].position

        # Keeping track of the particles throughout the algorithm run

            iteration_track = swarm[k].write_track_to_csv(pso_track, Iter,gbest_position=gbest_position,gbest_fitness=gbest_fitness)
            print(iteration_track)
            if(os.path.isfile(track_file)):
                pd.DataFrame(iteration_track).to_csv(track_file, mode='a', index=False, header=False)
            else:
                pd.DataFrame(iteration_track).to_csv(track_file, index=False)

        print("ITERR",Iter)
        show_particles(swarm,Iter,False,colors,"") 
        Iter += 1
    
    return gbest_position, gbest_fitness


if __name__ == '__main__':

    joint_config_index = pd.read_csv("Sorted_Joint_Configs_Indexs.csv")
    link_config_index = pd.read_csv("Sorted_Link_Configs_Indexs.csv")

    # population_size = 20 # common selection is between 20-50

    for i in range(2):
        track_file = '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/finalexperiment_track'+str(i)+'.csv'
        sim_res_file = '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/sim_res_file'+str(i)+'.csv'
        start_time = datetime.now()
        with open(track_file,"w") as f:
            cr = csv.writer(f,delimiter=";",lineterminator="\n")
            cr.writerow([start_time])

        population_size = 2
        seeds = [(j+1) for j in range(i*population_size,(i*population_size)+population_size)]
        print(seeds)
        print(seeds[0])
        print(seeds[1])
        gbest_position, gbest_fitness = pso(joint_config_indexs=joint_config_index,link_config_indexs=link_config_index, generation_num=2, population_size=population_size,w_type='CLDIW', jump_threshold=10, track_file=track_file, sim_res_file=sim_res_file, seeds=seeds)
        print("Final Result:  " + " Best fitness = Reached "+str(gbest_fitness[0])+" Clusters, "+"Min Manipulability: " +str(gbest_fitness[1])+ ", Best Arm = " +str(gbest_position.arm_name) + " , Best Joint index = " + str(gbest_position.joint_config_index)+ " , Best Link Index = " + str(gbest_position.link_config_index))
        
        end_time = datetime.now()
        with open(track_file,"a") as f:
            cr = csv.writer(f,delimiter=";",lineterminator="\n")
            cr.writerow([end_time])
