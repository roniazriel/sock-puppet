import os
import random
import numpy as np
import pandas as pd
import time
from matplotlib import pyplot as plt
from matplotlib import animation
# from tabulate import tabulate
from numpy import mean
from ros import Ros, MoveGroupPythonInterface, UrdfClass
from simulator import Simulator, one_at_a_time
from arm_creation import create_arm, UrdfClass

""" This is the framwork of the optimization process -  PSO implementation
    
    A particle in the swarm has an id number:
            particle =  {
                "id":p,"arm_name": arm_name,"joint_types": joint_types,"links": links, "pbest_score": score , 'pbest_pos':[joint_index,link_index],
                "velocity": velocity
            }
    
    His position is a robotic manipualator that is compossed by links configuration index and joint configuration index.
            'pbest_pos':[joint_index,link_index]
    
    The Velocity is the jump it will take to the next manipulator. If jump_neighborhood then the change will be in the link configuration index.
    otherwise, in the joint configuration.

    Neighborhood is considered as joint configuration, each of the link configuration will define a particle in the neighborhood.

"""
class Swarm:
    def __init__(self, joint_config_index,link_config_index, particles, velocities,population_size):
        ''' Swarm of particles
            :param joint_config_index: joint configuration index
            :param link_config_index: link configuration index
            :param particles: the particles in the swarm
            :param max_score: particles velocities
            :param population_size: number of particles in population
        '''
        self.joint_indexs = joint_config_index
        self.link_indexs = link_config_index
        self.population_size = population_size # int

        if particles is None:
            self.particles = self._initiate_particles()
        else:
            self.particles = particles # array of dict

        if velocities is None:
            self.velocities = [0 for i in range(self.population_size)] # array
        else:
            self.velocities = velocities


    def _initiate_particles(self):

        """
    :return: list of particles
    """
        particles = []
        for p in range(self.population_size):
            # Generate Joint Configuration index to start from (1-6836)
            joint_index = random.randint(1,6836)
            link_index = random.randint(1,456)

            joints = self.joint_indexs.loc[self.joint_indexs["Index"]==joint_index]
            links = self.link_indexs.loc[self.link_indexs["Index"]==link_index]

            joint_types=[joints['Joint'+str(i+1)+'_Type'].values[0] for i in range(6)]
            joint_axis=[joints['Joint'+str(i+1)+'_Axis'].values[0] for i in range(6)]
            links = [str(links['Link'+str(i+1)+'_Length'].values[0]) for i in range(6)]

            print("Joints types: ", joint_types)
            print("Joints axis: ", joint_axis)
            print("Links lengths: ", links)

            # Create URDF - the particle
            arm_name = create_arm(joint_types,joint_axis,links)
            print("Arm Name: ",arm_name)

            particle =  {
                "id":p,"arm_name": arm_name,"joint_types": joint_types,"links": links, "pbest_score": -1 , 'pbest_pos': np.array([joint_index, link_index]),
                "velocity": 0
            }

            particles.append(particle)

        return particles

class PSO:
    def __init__(self, population_size, generation,joint_config_index,link_config_index, max_score=None, max_group=4888, min_group=0):
        """
    :param population_size: number of particles in population
    :param generation: number of generations to run algorithm for
    :param max_score: score to stop algorithm once reached
    """

        if isinstance(population_size, int) and population_size > 0:
            self.population_size = population_size
        else:
            raise TypeError('population size must be a positive integer')

        if isinstance(generation, int) and generation > 0:
            self.generation = generation
        else:
            raise TypeError('generation must be a positive integer')

        if max_score is not None:
            if isinstance(max_score, (int, float)):
                self.max_score = float(max_score)
            else:
                raise TypeError('Maximum score must be a numeric type')

        self.swarm = Swarm(joint_config_index=joint_config_index,link_config_index=link_config_index, particles=None, velocities=None,population_size=self.population_size)


    def initiate_swarm_score(self):

        for p in self.swarm.particles:
            arm_name = p['arm_name']
            joint_types = p['joint_types']
            links = p['links']
            score = self._fitness_function(particle=p)
            p['pbest_score'] = score

        print(self.swarm.particles)
        return self.swarm.particles


    def _fitness_function(self, particle , result_file='/home/ar1/catkin_ws/src/sock-puppet/man_code/scripts/optimization_results11.csv'):
        """
    Returns objective function value of a state
    :param particle: a solution
    :return: objective function value of solution
    """
        
        result_file = result_file
        arm_name = particle['arm_name']
        joint_types = particle['joint_types']
        links = particle['links']

        simulation_db = pd.DataFrame(columns=["Arm ID","Point number", "Move duration", "Success", "Manipulability - mu","Manipulability - jacobian","Manipulability - cur pose","Manipulability - roni","Mid joint proximity",
                                            "Max Mid joint proximity","Sum Mid joint proximity","Sum Mid joint proximity- all joints"])
        
        
        one_at_a_time(6,arm_name,simulation_db,joint_types,links,result_file)
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

        return objective


    def _update_velocity(self, particle,current_position, gbest,jump_neighborhood, w_min=0.1, max=0.5, c=0.1):
        """
        pbest: [personal best Joint configuration, personal best Link configuration]
        gbest: [global best Joint configuration, global best Link configuration]
        """
        # Randomly generate r1, r2 and inertia weight from normal distribution
        r1 = random.uniform(0, max)
        r2 = random.uniform(0, max)
        
        # hyper parameters
        w = 0.729    # inertia
        c1 = 1.49445 # cognitive (particle)
        c2 = 1.49445 # social (swarm)

        # Calculate new velocity
        pbest = particle['pbest_pos']
        velocity = particle['velocity']
        if jump_neighborhood:
            new_velocity = w * velocity + c1 * r1 * (pbest[0] - current_position[0]) + c2 * r2 * (gbest[0] - current_position[0])
        else:
            new_velocity = w * velocity + c1 * r1 * (pbest - current_position) + c2 * r2 * (gbest- current_position)
            print(new_velocity)
        return new_velocity.astype(int)

    def _update_position(self, particle, velocity):
        # Move particles by adding velocity
        """
        Returns list of all members of neighborhood of current state, given self.current
        :return: list of members of neighborhood
        """
        # print("current: ", particle)
        # print("velocity", velocity)

        group_number = particle[0] + velocity
        # print("group number", group_number)
        if group_number >= self.max_group:
            group_list = [arm for arm in self.data if arm[0] == group_number or arm[0] == velocity]
        if group_number <= self.min_group:
            group_list = [arm for arm in self.data if arm[0] == self.min_group or arm[0] == velocity]
        else:
            group_list = [arm for arm in self.data if arm[0] == group_number or arm[0] == group_number + 1]
        if not bool(group_list):
            group_list = particle
        new_particle = group_list[random.randint(0, len(group_list) - 1)]
        return new_particle


    def pso_run(self, fitness_criterion=1, init=None):
        '''
        pbest_position: Best position (particle) for each initial particle
        pbest_fitness: Best objective score for each initial particle
        gbest_index: The index of the globally best particle
        gbest_position: Globally best particle
        '''
        # search tracking
        track = []
        track_score = []
        tracks = []
        track_scores = []
        # Initialisation
        # Population
        if init is None:
            particles = self._initiate_particles()
        else:
            particles = init
        track.append(particles)
        # Particles best position
        pbest_position = particles

        # Fitness
        pbest_fitness = [self._fitness_function(p) for p in particles]
        track_score.append(pbest_fitness)

        # Index of the best particle
        gbest_index = np.argmax(pbest_fitness)

        # Global best particle position
        gbest_position = pbest_position[gbest_index]

        # Velocity (starting from 0 speed)
        velocity = [0 for i in range(self.population)]

        # Loop for the number of generation
        for t in range(self.generation):
            tracks.append(particles)
            track_scores.append(pbest_fitness)
            # Stop if the average fitness value reached a predefined success criterion
            if np.average(pbest_fitness) <= fitness_criterion:
                break
            else:
                for n in range(self.population):
                    # Update the velocity of each particle
                    velocity[n] = self._update_velocity(particles[n], velocity[n], pbest_position[n], gbest_position)
                    # Move the particles to new position
                    particles[n] = self._update_position(particles[n], velocity[n])
            # Calculate the fitness value
            pbest_fitness = [self._fitness_function(p) for p in particles]
            # Find the index of the best particle
            gbest_index = np.argmax(pbest_fitness)
            # Update the position of the best particle
            gbest_position = pbest_position[gbest_index]

        # Print the results
        print('Global Best Position: ', gbest_position)
        print('Best Fitness Value: ', max(pbest_fitness))
        print('Average Particle Best Fitness Value: ', np.average(pbest_fitness))
        print('Number of Generation: ', t)
        return gbest_position, pbest_fitness, track, track_score, tracks, track_scores


if __name__ == '__main__':

    joint_config_index = pd.read_csv("Joint_Configurations_Indexs.csv")
    joint_config_index = joint_config_index.drop("Unnamed: 0", axis=1)
    link_config_index = pd.read_csv("Link_Configuration_Indexs.csv")
    link_config_index = link_config_index.drop("Unnamed: 0", axis=1)

    pso = PSO(population_size=1, generation=5,joint_config_index=joint_config_index,link_config_index =link_config_index)
    particles = pso.swarm.particles 

    # list_links = [d['links'] for d in particles]
    # list_joints = [d['joint_types'] for d in particles]
    # list_names = [d['arm_name'] for d in particles]

    # print(list_names)
    # print(list_joints)
    # print(list_links)

    particles = pso.initiate_swarm_score()
    print(particles)
    for p in particles:
        new_v = pso._update_velocity(p,p['pbest_pos'], np.array([11, 9]),jump_neighborhood=False)
        print(new_v)
    # for p in particles:
    #     arm_name = p['arm_name']
    #     joint_types = p['joint_types']
    #     links = p['links']

    #     pso._fitness_function(particle=p)
    # arm_name, joints_types, links = pso._initiate_particles()
    # obj = pso._fitness_function(arm_name, joints_types, links)
    # print(obj)









    # import data
    # all_data = pd.read_csv("grouped_data.csv")
    # all_data = all_data.drop("Unnamed: 0", axis=1)
    # columns_names = all_data.keys()

    # # Define configuration number for each robotic arm (index)
    # all_data['Configuration index'] = all_data.groupby(['Joint2 type_pitch', 'Joint2 type_pris', 'Joint2 type_roll',
    #                                                     'Joint3 type_pitch', 'Joint3 type_pris', 'Joint3 type_roll',
    #                                                     'Joint4 type_pitch', 'Joint4 type_pris', 'Joint4 type_roll',
    #                                                     'Joint5 type_pitch', 'Joint5 type_pris', 'Joint5 type_roll',
    #                                                     'Joint6 type_pitch', 'Joint6 type_pris', 'Joint6 type_roll',
    #                                                     'Joint2 axis_y', 'Joint2 axis_z', 'Joint3 axis_x',
    #                                                     'Joint3 axis_y',
    #                                                     'Joint3 axis_z', 'Joint4 axis_x', 'Joint4 axis_y',
    #                                                     'Joint4 axis_z',
    #                                                     'Joint5 axis_x', 'Joint5 axis_y', 'Joint5 axis_z',
    #                                                     'Joint6 axis_x',
    #                                                     'Joint6 axis_y', 'Joint6 axis_z']).ngroup()

    # all_data = all_data[['Configuration index', 'Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
    #                      'Link6 length', 'Joint2 type_pitch', 'Joint2 type_pris', 'Joint2 type_roll',
    #                      'Joint3 type_pitch', 'Joint3 type_pris', 'Joint3 type_roll',
    #                      'Joint4 type_pitch', 'Joint4 type_pris', 'Joint4 type_roll',
    #                      'Joint5 type_pitch', 'Joint5 type_pris', 'Joint5 type_roll',
    #                      'Joint6 type_pitch', 'Joint6 type_pris', 'Joint6 type_roll',
    #                      'Joint2 axis_y', 'Joint2 axis_z', 'Joint3 axis_x', 'Joint3 axis_y',
    #                      'Joint3 axis_z', 'Joint4 axis_x', 'Joint4 axis_y', 'Joint4 axis_z',
    #                      'Joint5 axis_x', 'Joint5 axis_y', 'Joint5 axis_z', 'Joint6 axis_x',
    #                      'Joint6 axis_y', 'Joint6 axis_z', 'Success_Rates', 'Manipulability_Rates']]

    # print("Maximum number of Configuration index: \n", all_data['Configuration index'].max())
    # print("Minimum number of Configuration index: \n", all_data['Configuration index'].min())

    # random.seed(456)
    # np.random.seed(456)

    # '''Initial Run'''
    # pso_search = PSO(all_data, population_size=50, generation=200)
    # best_sol, best_score, track, track_score, tracks, track_scores = pso_search.pso_run()
    # print("Best solution found: \n", best_sol)
    # print("Best objective function score: \n", max(best_score))
    # print("Track solution: \n", np.squeeze(track))
    # print("Score's Track: \n", np.squeeze(track_score))

    # df = pd.DataFrame(best_sol).T
    # df.columns = ['Configuration index', 'Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
    #               'Link6 length', 'Joint2 type_pitch', 'Joint2 type_pris', 'Joint2 type_roll',
    #               'Joint3 type_pitch', 'Joint3 type_pris', 'Joint3 type_roll',
    #               'Joint4 type_pitch', 'Joint4 type_pris', 'Joint4 type_roll',
    #               'Joint5 type_pitch', 'Joint5 type_pris', 'Joint5 type_roll',
    #               'Joint6 type_pitch', 'Joint6 type_pris', 'Joint6 type_roll',
    #               'Joint2 axis_y', 'Joint2 axis_z', 'Joint3 axis_x', 'Joint3 axis_y',
    #               'Joint3 axis_z', 'Joint4 axis_x', 'Joint4 axis_y', 'Joint4 axis_z',
    #               'Joint5 axis_x', 'Joint5 axis_y', 'Joint5 axis_z', 'Joint6 axis_x',
    #               'Joint6 axis_y', 'Joint6 axis_z', 'Success_Rates', 'Manipulability_Rates']
    # print(tabulate(df, headers='keys', tablefmt='psql'))

    # x = [i[0] for i in np.squeeze(track)]
    # print(x)
    # y = np.squeeze(track_score)
    # plt.plot(x, y, 'ro')
    # plt.title("Observed robotic configurations in first generation - PSO")
    # plt.xlabel("Configuration Index")
    # plt.ylabel("Objective Function Value")
    # plt.show()

    # x = [i[0][0] for i in np.squeeze(tracks)]
    # print(x)
    # y = np.squeeze(track_scores)
    # plt.plot(x, y, 'ro')
    # plt.title("Observed robotic configurations in all generations - PSO")
    # plt.xlabel("Configuration Index")
    # plt.ylabel("Objective Function Value")
    # plt.show()

    # ''' hyper parameter tuning'''
    # generation_num = [50, 100, 150, 200]
    # population_num = [30, 50, 70, 90]
    # comb_array = []
    # av_res_array = []
    # for i in range(len(generation_num)):
    #     for j in range(len(population_num)):
    #         comb = "generation: " + str(generation_num[i]) + "population: " + str(population_num[j])
    #         comb_array.append(comb)
    #         res_array = []
    #         for k in range(10):
    #             pso_search = PSO(all_data, population_size=population_num[j], generation=generation_num[i])
    #             best_sol, best_score, track, track_score, tracks, track_scores = pso_search.pso_run()
    #             df = pd.DataFrame(best_sol).T
    #             df.columns = ['Configuration index', 'Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
    #                           'Link6 length', 'Joint2 type_pitch', 'Joint2 type_pris', 'Joint2 type_roll',
    #                           'Joint3 type_pitch', 'Joint3 type_pris', 'Joint3 type_roll',
    #                           'Joint4 type_pitch', 'Joint4 type_pris', 'Joint4 type_roll',
    #                           'Joint5 type_pitch', 'Joint5 type_pris', 'Joint5 type_roll',
    #                           'Joint6 type_pitch', 'Joint6 type_pris', 'Joint6 type_roll',
    #                           'Joint2 axis_y', 'Joint2 axis_z', 'Joint3 axis_x', 'Joint3 axis_y',
    #                           'Joint3 axis_z', 'Joint4 axis_x', 'Joint4 axis_y', 'Joint4 axis_z',
    #                           'Joint5 axis_x', 'Joint5 axis_y', 'Joint5 axis_z', 'Joint6 axis_x',
    #                           'Joint6 axis_y', 'Joint6 axis_z', 'Success_Rates', 'Manipulability_Rates']
    #             print("Best solution found for population size " + str(
    #                 population_num[j]) + " and generation number " + str(generation_num[i]) + " is: \n")
    #             print(tabulate(df, headers='keys', tablefmt='psql'))
    #             print("Best objective function score for " + str(population_num[j]) + " and generation number " + str(
    #                 generation_num[i]) + " is: \n", max(best_score))
    #             res_array.append(max(best_score))
    #         av_res_array.append(mean(res_array))
    #         # print("Track solution for " + str(population_num[j])  + " and generation number "+ str(generation_num[i]) + " is: \n", np.squeeze(track))
    #         # print("Score's Track for " + str(population_num[j])  + " and generation number "+ str(generation_num[i]) + " is: \n", np.squeeze(track_score))

    #         df = pd.DataFrame(best_sol).T
    #         df.columns = ['Configuration index', 'Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
    #                       'Link6 length', 'Joint2 type_pitch', 'Joint2 type_pris', 'Joint2 type_roll',
    #                       'Joint3 type_pitch', 'Joint3 type_pris', 'Joint3 type_roll',
    #                       'Joint4 type_pitch', 'Joint4 type_pris', 'Joint4 type_roll',
    #                       'Joint5 type_pitch', 'Joint5 type_pris', 'Joint5 type_roll',
    #                       'Joint6 type_pitch', 'Joint6 type_pris', 'Joint6 type_roll',
    #                       'Joint2 axis_y', 'Joint2 axis_z', 'Joint3 axis_x', 'Joint3 axis_y',
    #                       'Joint3 axis_z', 'Joint4 axis_x', 'Joint4 axis_y', 'Joint4 axis_z',
    #                       'Joint5 axis_x', 'Joint5 axis_y', 'Joint5 axis_z', 'Joint6 axis_x',
    #                       'Joint6 axis_y', 'Joint6 axis_z', 'Success_Rates', 'Manipulability_Rates']
    #         print("The chosen solution in table format: \n")
    #         print(tabulate(df, headers='keys', tablefmt='psql'))

    #         x = [i[0] for i in np.squeeze(track)]
    #         print(x)
    #         y = np.squeeze(track_score)
    #         plt.plot(x, y, 'ro')
    #         plt.title("Observed robotic configurations in first generation - PSO")
    #         plt.xlabel("Configuration Index")
    #         plt.ylabel("Objective Function Value")
    #         plt.show()

    # greed_best_score_index = np.argmax(av_res_array)
    # best_comb = comb_array[greed_best_score_index]
    # best_score_found = av_res_array[greed_best_score_index]
    # print("Best combination found: \n", best_comb)
    # print("Best avarage score achieved: \n", best_score_found)

    # '''Start from the same initial solution'''
    # starting_points = all_data.sample(n=90).values.squeeze().tolist()
    # print(starting_points)

    # pso_search = PSO(all_data, population_size=90, generation=150)
    # best_sol, best_score, track, track_score, tracks, track_scores = pso_search.pso_run(init=starting_points)
    # print("The score obtained by PSO: ", max(best_score))
    # print("The selected arm configurations: ", best_sol)

    # x= [i for i in range(90)]
    # y= best_score
    # plt.plot(x,y)
    # plt.title("Scores of the last generation - PSO")
    # plt.xlabel("Iteration number")
    # plt.ylabel("Objective function score")
    # plt.show()
