import numpy as np
import copy
import asyncio
import sys

RUN = int(sys.argv[1])
print("RUN index:", RUN)

csv_file = "pso_" + str(RUN) + ".csv"
with open(csv_file, 'a') as f:
    f.write("environment_type,num_agents,seed,inertia_weight,cognitive_weight,social_weight,simulation_time,agents_escaped\n")

########################## PARAMETER SETTINGS ##########################

# Fixed
dimensions = [10, 10] # information already present in the scenarios
surface = dimensions[0] * dimensions[1]


# Environment parameters
environment_types = ["two_doors", "slalom"]
seed = [1, 2, 5]
agents_wrt_surface = [50, 90, 30] 


# PSO parameters
inertia_weight = [0.4, 0.6, 0.8]
cognitive_weight = [1.0, 1.2, 1.5]
social_weight = [1.5, 1.8, 2.0]

########################## END PARAMETER SETTINGS ##########################

from main import *
from parser.config import Config
#from visualization.visualizer import Visualizer

async def run_simulation(env_to_test, configVal, visualizer):
    viz_task = asyncio.create_task(visualization_loop(visualizer))

    try:
        sim_time, agents_remaining = await main_program(
            env_to_test, configVal, visualizer
        )
    finally:
        viz_task.cancel()
        try:
            await viz_task
        except asyncio.CancelledError:
            pass

    return sim_time, agents_remaining

run_count = 0
run_details = []
for env in environment_types:           # 2
    for s in seed:             # 3 
        for agents in agents_wrt_surface:         # 3
            run_details.append((env, s, agents)) 
            run_count += 1
         
print("Total runs:", run_count)
if RUN >= run_count:
    raise ValueError("RUN index out of range. Max RUN is " + str(run_count - 1))

env_type, s, agents = run_details[RUN]

count = 0
tot = len(inertia_weight) * len(cognitive_weight) * len(social_weight) 

np.random.seed(s)

env = get_scenario_by_name(env_type, agents = [agents, "pso-local"]) 
if env is None:
    raise ValueError("Scenario " + str(environment_types) + " not recognized.")


    
for w in inertia_weight:                      # 3
    for c1 in cognitive_weight:                    # 3
        for c2 in social_weight:                       # 3
                           
            start = time.time()
                        
            env_to_test = copy.deepcopy(env)
            if len(env_to_test.agents) != agents:
                raise ValueError("Number of agents in the environment does not match the config file.")
                            
            # step 1: create config file associated to the parameters
            configVal = Config()
            configVal.algorithm = "pso-local"
            configVal.world_type = env_type
            
            configVal.random_seed = s
            configVal.dt = 0.01
            configVal.num_agents = agents
            configVal.neighborhood_radius = 10.0
            
            configVal.W = w
            configVal.C1 = c1
            configVal.C2 = c2
            
            configVal.world_name = ""
            
            configVal.visualization = False

            # configVal.window_name = "Crowd Simulation"
            # configVal.visualization_dimensions = [1200, 800]
            
            # configVal.padding = 50
            # configVal.right_border = 300
            # configVal.left_border = 300
            # configVal.top_border = 0
            # configVal.bottom_border = 120
            # configVal.background_color = [0, 0, 0, 255]
            # configVal.agent_color = [243, 255, 20, 255]
            # configVal.wall_color = [200, 200, 200, 255]
            # configVal.exit_color = [57, 255, 20, 255]
            # configVal.title_color = [255, 0, 0, 255]
            # configVal.text_color = [255, 255, 255, 255]
            # if configVal.visualization:
            #     visualizer = Visualizer(environment=env_to_test, config=configVal)

            # step 2: call main with the configVal file
            sim_time, agents_remaining = asyncio.run(
               main_program(env_to_test, configVal)
            )
            # sim_time, agents_remaining = asyncio.run(
            #     run_simulation(env_to_test, configVal, visualizer)
            # )
            #sim_time, agents_remaining = main_program(env_to_test, configVal) #asyncio.run(initialize_main(configVal))
            #print(f"Simulation time: {sim_time}, Saved_agents: {num_agent - agents_remaining}/{num_agent}")
            
            with open(csv_file, 'a') as f:
                f.write(f"{env_type},{agents},{s},{w},{c1},{c2},{sim_time},{agents_remaining}\n")
        
            end = time.time()
            count += 1
            print(f"[RUN {RUN}] Simulation {count}/{tot} completed in {end - start} seconds.")
            




