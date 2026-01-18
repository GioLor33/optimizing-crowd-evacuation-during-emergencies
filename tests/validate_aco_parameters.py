import numpy as np
import os

# TODO: save data into a csv file

csv_file = "aco_parameter_validation_results.csv"
if not os.path.exists(csv_file):
    with open(csv_file, 'w') as f:
        f.write("environment_type,num_agents,seed,graph_type,num_nodes,alpha,beta,evaporation_rate,num_ants,num_iterations,simulation_time,agents_escaped\n")

########################## PARAMETER SETTINGS ##########################

# Fixed
dimensions = [10, 10] # information already present in the scenarios
surface = dimensions[0] * dimensions[1]


# Environment parameters
environment_types = ["two_doors", "slalom", "empty"]
seed = [1, 2, 3]
agents_wrt_surface = [int(surface * x) for x in [0.5, 1, 2]]  # 50, 100, 200 agents for 10x10 env


# ACO parameters
surface = dimensions[0] * dimensions[1]
graph_type = ["grid", "PRM"]
nodes_wrt_surface = [int(surface * x) for x in [0.5, 1, 2]]  # 50, 100, 200 nodes for 10x10 env
    
alpha = [0.5, 1.0, 2.0, 5.0]
beta = [0.5, 1.0, 2.0, 5.0]
evaporation_rate = [0.05, 0.1, 0.2, 0.5]
num_ants = [int(x + x * y) for x in nodes_wrt_surface for y in [0.01, 0.05, 0.1]]  # +1%, +5%, +10% of the number of nodes (for 10x10)
num_iterations_wrt_surface = [int(surface * x) for x in [1, 2, 5]]  # 100, 200, 500 iterations for 10x10 env


########################## END PARAMETER SETTINGS ##########################

from main import *
from parser.config import Config

count = 0

for env_type in environment_types:
 for num_agent in agents_wrt_surface:
  for s in seed:
   for g in graph_type:
    for nodes in nodes_wrt_surface:
     for a in alpha:
      for b in beta:
       for evap in evaporation_rate:
        for ants in num_ants:
         for iters in num_iterations_wrt_surface:
             
            start = time.time()
             
            # step 1: create config file associated to the parameters
            config = Config()

            config.algorithm = "aco"
            config.random_seed = seed
            config.dt = 0.01
            
            config.num_ants = ants
            config.num_iterations = iters
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
            sim_time, agents_remaining = asyncio.run(initialize_main(config))
            print(f"Simulation time: {sim_time}, Saved_agents: {num_agent - agents_remaining}/{num_agent}")
            
            with open(csv_file, 'a') as f:
                f.write(f"{env_type},{num_agent},{s},{g},{nodes},{a},{b},{evap},{ants},{iters},{sim_time},{num_agent - agents_remaining}\n")
        
            end = time.time()
            print(f"Simulation completed in {end - start} seconds.")
            
            count += 1
            if count == 5:
                raise ValueError("Stopping after 5 simulations for testing purposes.")