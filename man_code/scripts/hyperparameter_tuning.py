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
from sklearn.model_selection import ParameterGrid
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
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
    def __init__(self,Pid, joint_config_indexs,link_config_indexs,jump_threshold,seed,data):
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
        self.data = data
        self.id = Pid
        self.seed = seed
        self.jump_threshold = jump_threshold

        # csv files with the feasible solutions
        self.joint_indexs = joint_config_indexs
        self.link_indexs = link_config_indexs

        # initialize joint config velocity of the particle with 0.0 value
        self.max_joint_config_velocity = 6836 # max velocity = max x
        R = np.random.RandomState(self.seed)
        self.joint_config_velocity = R.randint(2,10)

        # initialize an indicator for switching between joint configurations 
        self.jump_joint_config = 1

        # randomly initialize position of the particle
        self._update_position(initiate=True ) # initialize also best particle position to initial position
        self._fitness_function(pbest=True) # initialize also best particle fitness of the particle


    
    def _fitness_function(self,pbest=False, result_file='/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/optimization_results_tests.csv'):
        """ Returns objective function value of a state """
        
        if pbest:
            arm_name = self.pbest_position.arm_name
            joint_types = self.pbest_position.joint_types
            links = self.pbest_position.links

        else:
            
            arm_name = self.position.arm_name
            joint_types = self.position.joint_types
            links = self.position.links

        ans = self.data.groupby('Arm_ID').aggregate({'Success_Rates': 'sum', 'Min_Manipulability': 'min'}).reset_index().rename(columns={'Success_Rates': 'Reachability'})

        Reachability = (ans.loc[(ans['Arm_ID'] == arm_name)]['Reachability'].values[0])*10
        Min_Manipulability = ans.loc[(ans['Arm_ID'] == arm_name)]['Min_Manipulability'].values[0]

        if pd.isna(Min_Manipulability):
            Min_Manipulability = 0
        
        objective = [Reachability,Min_Manipulability]

        if pbest:
            self.pbest_fitness = objective
            self.position_fitness = objective
        else:
            self.position_fitness = objective

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

    def _update_velocity(self, gbest_position, w,c1,c2):
        """
        pbest: [personal best Joint configuration, personal best Link configuration]
        gbest: [global best Joint configuration, global best Link configuration]
        """
        # Randomly generate r1, r2 and inertia weight from normal distribution
        r1 = random.random()
        r2 = random.random()

        # Calculate new velocity
        self.jump_joint_config +=1
        
        if self.jump_joint_config%self.jump_threshold==0:           
            new_joint_velocity = w * self.joint_config_velocity + c1 * r1 * (self.pbest_position.joint_config_index - self.position.joint_config_index) + c2 * r2 * (gbest_position.joint_config_index - self.position.joint_config_index)
            
            if abs(new_joint_velocity) <= self.max_joint_config_velocity:
                self.joint_config_velocity = int(new_joint_velocity)
            else:
                self.joint_config_velocity = self.max_joint_config_velocity
            # print("new joint velocity: ", new_joint_velocity)

    def _update_position(self, initiate, pbest=False):

        if initiate:
            arm = self.data.sample(n=1, random_state = self.seed)
            joint_index = arm["Joint_Config_Index"].values[0]
            link_index = arm["Link_Config_Index"].values[0]
        else:
            joint_index = self.position.joint_config_index
            arm = self.data.loc[self.data["Joint_Config_Index"]==joint_index].sample(1)
            link_index = arm["Link_Config_Index"].values[0]

        # If the threshold value was reached - change the position of the joints index
        if self.jump_joint_config%self.jump_threshold==0: 
            # position has to be between [min joint_index , max joint_index ]
            if self.joint_config_velocity + joint_index < 1:
                joint_index = 1
            elif self.joint_config_velocity + joint_index > 6836:
                joint_index = 6836
            else:
                joint_index = joint_index + self.joint_config_velocity

        # Get new particle info
        joints = self.joint_indexs.loc[self.joint_indexs["Index"]==joint_index]
        links = self.link_indexs.loc[self.link_indexs["Index"]==link_index]

        joint_types=[joints['Joint'+str(i+1)+'_Type'].values[0] for i in range(6)]
        joint_axis=[joints['Joint'+str(i+1)+'_Axis'].values[0] for i in range(6)]
        links_lengths = [str(links['Link'+str(i+1)+'_Length'].values[0]) for i in range(6)]

        # print("Joints types: ", joint_types)
        # print("Joints axis: ", joint_axis)
        # print("Links lengths: ", links_lengths)

        # Create URDF - the particle
        arm_name = arm["Arm_ID"].values[0]
        # print("Arm Name: ",arm_name)

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
    plt.close(fig)

def pso(joint_config_indexs,link_config_indexs, generation_num, population_size, jump_threshold, w_type ,c1,c2, track_file,data,replication):
        
    # create n random particles
    swarm = [Particle(i,joint_config_indexs,link_config_indexs,jump_threshold, i*replication ,data) for i in range(population_size)]
 
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
        pso_track = pd.DataFrame(columns=["Iteration_number","ParticleID","Joint_Velocity","Current_Link_Index", 
        "Current_Joint_Index", "Current_Joints_Types","Current_Joints_Axis","Current_Links_Lengths","Current_Fitness", "pbest_Link_Index", 
        "pbest_Joint_Index", "pbest_Joints_Types","pbest_Joints_Axis","pbest_Links_Lengths","pbest_Fitness"])

        initialization_track = swarm[i].write_track_to_csv(pso_track, "Initialization",gbest_position=gbest_position,gbest_fitness=gbest_fitness)

        if(os.path.isfile(track_file)):
            pd.DataFrame(initialization_track).to_csv(track_file, mode='a', index=False, header=False)
        else:
            pd.DataFrame(initialization_track).to_csv(track_file, index=False)

    # colors = np.random.random(len(swarm))
    # show_particles(swarm,0,True,colors,"")

    # main loop of pso
    Iter = 0
    gbset_iteration = 0
    # generate random chaotic number
    zk = random.random()
    # initialization rules
    if ((zk==0.0) or (zk==0.25) or (zk==0.5) or (zk == 0.75) or (zk==1.0)) :
        zk = random.random()

    # Plotting prepartion
    # fig = plt.figure(figsize=(10, 10))
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.set_xlim([1, 500])
    # ax.set_ylim([1, 7000])
    # ax.set_title("Comb: ")
    # ax.set_xlabel("Link Configuration Index")
    # ax.set_ylabel("Joint Configuration Index")

    # arms = data.sample(n=1000)

    # joint_indexs = arms["Joint_Config_Index"].to_numpy()
    # link_indexs = arms["Link_Config_Index"].to_numpy()
    # manipulability = arms["Min_Manipulability"].to_numpy()
    # X = link_indexs
    # Y = joint_indexs
    # # X, Y = np.meshgrid(x, y)
    # Z= manipulability
    # ax.plot_trisurf(X, Y, Z, color='r', linewidth=0.2)
    # plt.show()
    # Animation image placeholder
    # images = []

    while Iter < generation_num:
        
        # after every 10 iterations
        # print iteration number and best fitness value so far
        if Iter % 50 == 0 and Iter > 1:
          print("Iter = " + str(Iter) + " Best fitness = Reached "+str(gbest_fitness[0])+" Clusters, "+"Min Manipulability: " +str(gbest_fitness[1])+ 
        ", Best Arm = " +str(gbest_position.arm_name) + " , Best Joint index = " + str(gbest_position.joint_config_index)+ " , Best Link Index = " + str(gbest_position.link_config_index))

        # calculate inertia:
        if w_type == 'LDWI':
            w = Particle.LDIW(max_iter=generation_num, current_iter=Iter)
        elif w_type == 'CLDIW':
            w,zk= Particle.CLDIW(max_iter=generation_num, current_iter=Iter, zk=zk)
        else:
            w=0.729

        for k in range(population_size): # process each particle
            swarm[k]._update_velocity(gbest_position, w,c1,c2)
          # compute new position using new velocity
            swarm[k]._update_position(initiate=False)

          # compute fitness of new position
            swarm[k]._fitness_function()

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
                gbset_iteration=Iter

            elif swarm[k].position_fitness[0] == gbest_fitness[0]:
                if swarm[k].position_fitness[1] > gbest_fitness[1]:
                    gbest_fitness = swarm[k].position_fitness
                    gbest_position = swarm[k].position
                    gbset_iteration=Iter

        # Keeping track of the particles throughout the algorithm run

            iteration_track = swarm[k].write_track_to_csv(pso_track, Iter,gbest_position=gbest_position,gbest_fitness=gbest_fitness)
            # print(iteration_track)
            if(os.path.isfile(track_file)):
                pd.DataFrame(iteration_track).to_csv(track_file, mode='a', index=False, header=False)
            else:
                pd.DataFrame(iteration_track).to_csv(track_file, index=False)

        # show_particles(swarm,Iter,False,colors,"") 
        
        # Add plot for each generation (within the generation for-loop)
        # links_indexses=[swarm[n].position.link_config_index for n in range(population_size)]
        # joint_indexses=[swarm[n].position.joint_config_index for n in range(population_size)]
        # names = [swarm[n].id for n in range(population_size)]
        
        # # Generate the animation image and save
        # image = ax.scatter3D(links_indexses,joint_indexses,
        #                      [swarm[n].position_fitness[1] for n in range(population_size)], c=colors)
        # images.append([image])
        
        Iter += 1

    # animated_image = animation.ArtistAnimation(fig, images)
    # plt.show()
    # animated_image.save('./pso_simple.gif', writer='pillow')
    return gbest_position, gbest_fitness, gbset_iteration


if __name__ == '__main__':

    joint_config_index = pd.read_csv("Sorted_Joint_Configs_Indexs.csv")
    link_config_index = pd.read_csv("Sorted_Link_Configs_Indexs.csv")
    data = pd.read_csv("6dof_configs_indexes.csv")
    # population_size = 20 # common selection is between 20-50
    results_file = '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/Hparameter_optimization_results.csv'
    
    #HYPER PARAMETERS
    cs = [1.49445]
    jumps = [5,10,15]
    inertia_types = ['LDIW','CLDIW','CIW']
    generations = [200]
    population_sizes = [20,50]

    param_grid = {'c': cs, 'jump': jumps, 'inertia_type':inertia_types,'generation':generations, 'population_size':population_sizes}
    combs = ParameterGrid(param_grid)
    print(len(list(combs)))
    comb_num = 1
    for comb in list(combs):
        for rep in range(30):
            print("CURRENT COMBINATION: ",comb)
            print("REPLICATION NUMBER: " +str(rep+1)) 

            track_file = '/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/Hparameter_optimization_track'+str(comb_num)+'.csv'
            tuning_db = pd.DataFrame(columns=["Replication","learning_factors","jump_threshold", "inertia_type", "generation","population_size","gbest_Arm_name","gbest_Link_Index","gbest_Joint_Index",
            "gbest_Joints_Types","gbest_Joints_Axis","gbest_Links_Lengths","gbest_Fitness_Reachability","gbest_Fitness_Manipulability","gbset_iteration"])    
            
            gbest_position, gbest_fitness, gbset_iteration = pso(joint_config_indexs=joint_config_index,link_config_indexs=link_config_index, generation_num=comb['generation'], 
                population_size=comb['population_size'], jump_threshold=comb['jump'],w_type=comb['inertia_type'] ,c1=comb['c'],c2=comb['c'], track_file=track_file, data=data,replication=rep+1)

            print("Final Result:  " + " Best fitness = Reached "+str(gbest_fitness[0])+" Clusters, "+"Min Manipulability: " +str(gbest_fitness[1])+ 
                ", Best Arm = " +str(gbest_position.arm_name) + " , Best Joint index = " + str(gbest_position.joint_config_index)+ " , Best Link Index = " + str(gbest_position.link_config_index))

            tuning_db = tuning_db.append({"Replication":rep+1,"learning_factors":comb['c'],"jump_threshold":comb['jump'], "inertia_type":comb['inertia_type'], "generation":comb['generation'],
                "population_size":comb['population_size'],"gbest_Arm_name":gbest_position.arm_name,"gbest_Link_Index":gbest_position.link_config_index,"gbest_Joint_Index":gbest_position.joint_config_index,
                "gbest_Joints_Types":gbest_position.joint_types,"gbest_Joints_Axis":gbest_position.joint_axis,"gbest_Links_Lengths":gbest_position.links,
                "gbest_Fitness_Reachability":gbest_fitness[0],"gbest_Fitness_Manipulability":gbest_fitness[1],"gbset_iteration":gbset_iteration},ignore_index=True)

            if(os.path.isfile(results_file)):
                pd.DataFrame(tuning_db).to_csv(results_file, mode='a', index=False, header=False)
            else:
                pd.DataFrame(tuning_db).to_csv(results_file, index=False)

        comb_num += 1
            #####number of iteration found the best?
