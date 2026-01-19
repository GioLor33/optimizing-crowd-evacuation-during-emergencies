import numpy as np
import asyncio
import time
import matplotlib
matplotlib.use('TkAgg')


from environments.environment import Environment
from parser.config import Config
from environments.scenarios import get_scenario_by_name

config = Config("resources/config.yaml")
if config.visualization:
    from visualization.visualizer import Visualizer

async def visualization_loop(visualizer):
    assert isinstance(visualizer, Visualizer)
    while visualizer.window_is_open() and visualizer.on:
        visualizer.create_drawing()
        await asyncio.sleep(0)  # yield control

async def main_program(world, config, visualizer=None):
    if visualizer is not None:
        assert isinstance(visualizer, Visualizer)
    loop = asyncio.get_running_loop()
    
    num_agents = config.num_agents 
    dt = config.dt
    
    if config.algorithm == "boids":
        from boids_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
        print("Starting Boids algorithm simulation with " + str(num_agents) + " agents.")
    elif config.algorithm == "aco":
        from aco_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
        
        if visualizer is not None:
            visualizer.associate_graph(sim.aco_env)
            visualizer.enable_graph()
            
        # print("Starting ACO algorithm simulation with " + str(num_agents) + " agents.")
    elif config.algorithm == "pso-local":
        from pso_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
    else:
        raise ValueError("Algorithm " + str(config.algorithm) + " not recognized.")
        exit(1)
    
    print("Algorithm finished. Simulation started")
    while world.simulation_time < 30:  # Main executions
        start = time.time()
        world.simulation_start_time = start
        sim.update(dt)
        end = time.time()
        
        if (end - start) < dt:
            await asyncio.sleep(dt - (end - start))
            #time.sleep(dt - (end - start))
            
        #time.sleep(0.5)
        
        if len(sim.agents_escaped) == num_agents:
            break
        await asyncio.sleep(0)
        
    #print("All agents have evacuated.")
    return world.simulation_time, config.num_agents - len(sim.agents_escaped)
    
    # TODO: display "Simulation Complete" message on visualizer
        
        
async def initialize_main(config : Config):
    
    seed = config.random_seed
    if seed is not None:
        np.random.seed(seed)
    
    print("Creating the environment and adding the agents...")
    if config.world_type == "custom":
        env = Environment(
            name=config.world_name,
            dimensions=config.world_dimensions,
            exits=config.exits,
            walls=config.walls,
            agents=[config.num_agents, config.algorithm], 
            config = config
            #agents_spawn_method=config.spawn_agent_method
        )
    else:
        env = get_scenario_by_name(config.world_type, agents = [config.num_agents, config.algorithm], config = config)
        if env is None:
            raise ValueError("Scenario " + str(config.world_type) + " not recognized.")
        
    if config.visualization:
        visualizer = Visualizer(environment=env, config=config)

        await asyncio.gather(
            visualization_loop(visualizer),
            main_program(env, config, visualizer),
        )

    else:
        results = await main_program(env, config)
        
    return results

if __name__ == "__main__":
    asyncio.run(initialize_main(config))
    