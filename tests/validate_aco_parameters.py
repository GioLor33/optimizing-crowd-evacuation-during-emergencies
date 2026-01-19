import numpy as np
import copy
import asyncio

csv_file = "aco_parameter_validation_results.csv"
with open(csv_file, 'a') as f:
    f.write("environment_type,num_agents,seed,graph_type,num_nodes,alpha,beta,evaporation_rate,num_ants,num_iterations,simulation_time,agents_escaped\n")

########################## PARAMETER SETTINGS ##########################

# Fixed
dimensions = [10, 10] # information already present in the scenarios
surface = dimensions[0] * dimensions[1]


# Environment parameters
environment_types = ["two_doors", "slalom"]
seed = [1, 2, 5]
agents_wrt_surface = [50, 90, 30] 


# ACO parameters
surface = dimensions[0] * dimensions[1]
graph_type = ["grid", "PRM"]
nodes_wrt_surface = [int(surface * x) for x in [0.5, 1, 2]]  # 50, 100, 200 nodes for 10x10 env
    
alpha = [0.5, 1.0, 2.0]
beta = [1.5, 2.0, 2.5]
evaporation_rate = [0.05, 0.1, 0.2, 0.5]
num_ants = [0.05]  # +1%, +5%, +10% of the number of nodes (for 10x10)
num_iterations_wrt_surface = [3]  # 100, 200, 500 iterations for 10x10 env


########################## END PARAMETER SETTINGS ##########################

from main import *
from parser.config import Config

env_type = "two_doors"          # "slalom" on Pino
g = "grid"                      # "two_doors" && "PRM" still missing
agents_wrt_surface = [90, 30] 

count = 0
tot = len(nodes_wrt_surface) * len(alpha) * len(beta) * len(evaporation_rate) * len(num_ants) * len(num_iterations_wrt_surface) * len(agents_wrt_surface) * len(seed)

for num_agent in agents_wrt_surface:               # 3
    for s in seed:                                 # 3
        
        np.random.seed(s)
        
        env = get_scenario_by_name(env_type, agents = [num_agent, "aco"])
        if env is None:
            raise ValueError("Scenario " + str(env_type) + " not recognized.")
        
        for nodes in nodes_wrt_surface:                 # 3
            for a in alpha:                                # 3
                for b in beta:                                # 3
                    for evap in evaporation_rate:                # 4
                        for ants in num_ants:                       # 1    
                            for iters in num_iterations_wrt_surface:   # 1
                                
                                start = time.time()
                                
                                env_to_test = copy.deepcopy(env)
                                if len(env_to_test.agents) != num_agent:
                                    raise ValueError("Number of agents in the environment does not match the config file.")
                                    
                                # step 1: create config file associated to the parameters
                                config = Config()

                                config.algorithm = "aco"
                                config.random_seed = seed
                                config.dt = 0.01
                                
                                config.num_ants = int(nodes + nodes * ants)
                                config.num_iterations = iters * nodes
                                config.alpha = a
                                config.beta = b
                                config.evaporation_rate = evap
                                config.graph_type = g
                                if g == "PRM":
                                    config.n = nodes
                                    config.k_connectivity = 8
                                else:
                                    grid_size = int(np.sqrt(nodes))
                                    config.n = grid_size
                                    config.m = grid_size
                                    config.k_connectivity = 2
                                
                                config.world_name = ""
                                config.world_type = env_type
                                config.num_agents = num_agent
                                
                                config.visualization = False

                                # step 2: call main with the config file
                                sim_time, agents_remaining = asyncio.run(
                                    main_program(env_to_test, config)
                                )
                                #sim_time, agents_remaining = main_program(env_to_test, config) #asyncio.run(initialize_main(config))
                                #print(f"Simulation time: {sim_time}, Saved_agents: {num_agent - agents_remaining}/{num_agent}")
                                
                                with open(csv_file, 'a') as f:
                                    f.write(f"{env_type},{num_agent},{s},{g},{nodes},{a},{b},{evap},{ants},{iters},{sim_time},{num_agent - agents_remaining}\n")
                            
                                end = time.time()
                                count += 1
                                print(f"Simulation {count}/{tot} completed in {end - start} seconds.")
