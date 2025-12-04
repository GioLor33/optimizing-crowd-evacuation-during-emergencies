import numpy as np
import asyncio
import time

from visualization.visualizer import Visualizer
#from visualization.visualizerV2 import Visualizer
from environments.environment import Environment
from parser.config import Config

async def visualization_loop(visualizer):
    assert isinstance(visualizer, Visualizer)
    while visualizer.window_is_open() and visualizer.on:
        visualizer.create_drawing()
        await asyncio.sleep(0)  # yield control

async def main_program(visualizer, world, config):
    assert isinstance(visualizer, Visualizer)
    loop = asyncio.get_running_loop()
    
    num_agents = config.num_agents 
    dt = config.dt
    seed = config.random_seed
    if seed is not None:
        np.random.seed(seed)
        print(f"Random seed set to {seed}.")
    
    if config.algorithm == "boids-without-panic":
        from boids_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
        print("Starting Boids algorithm simulation with " + str(num_agents) + " agents.")
    if config.algorithm == "aco":
        from aco_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
        visualizer.associate_graph(sim.aco_env.nodes)
        visualizer.enable_graph()
        print("Starting ACO algorithm simulation with " + str(num_agents) + " agents.")
    else:
        raise ValueError("Algorithm " + str(config.algorithm) + " not recognized.")
        exit(1)
    
    while True:  # Main executions
        start = time.time()
        world.simulation_start_time = start
        sim.update(dt)
        end = time.time()
        
        if (end - start) < dt:
            time.sleep(dt - (end - start))
            
        #time.sleep(0.2)
            
        if len(sim.agents_escaped) == num_agents:
            break
        await asyncio.sleep(0)
        
    print("All agents have evacuated.")
    # TODO: display "Simulation Complete" message on visualizer
        
        
async def initialize_main():
    
    config = Config("resources/config_aco.yaml")
    
    env = Environment(
        name=config.world_name,
        dimensions=config.world_dimensions,
        exits=config.exits,
        walls=config.walls
    )
    
    if config.visualization:
        visualizer = Visualizer(environment=env, config=config)

        await asyncio.gather(
            visualization_loop(visualizer),
            main_program(visualizer, env, config),
        )
        
    else:
        await main_program(None, env)

if __name__ == "__main__":
    asyncio.run(initialize_main())
    