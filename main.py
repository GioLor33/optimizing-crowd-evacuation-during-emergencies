import numpy as np
import asyncio
import time

from environments.environment import Environment
from parser.config import Config
from environments.scenarios import get_scenario_by_name

config = Config("resources/config.yaml")
if config.visualization:
    from visualization.visualizer import Visualizer

async def visualization_loop(visualizer):
    assert isinstance(visualizer, Visualizer)
    while visualizer.window_is_open():
        while not visualizer.on:
            visualizer.spawn_algorithm_loading()
            await asyncio.sleep(0)
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
    elif config.algorithm == "pso":
        from pso_algorithm.crowdSimulator import CrowdSimulator
        sim = CrowdSimulator(world, config = config)
    else:
        raise ValueError("Algorithm " + str(config.algorithm) + " not recognized.")
    
    print("Simulation started")
    if visualizer is not None:
        visualizer.on = True
        await asyncio.sleep(0)
        
    while world.simulation_time < max(10, config.num_agents*1.5):  # Main executions

        if visualizer is not None:
            if not visualizer.play:
                await asyncio.sleep(0)
                continue
        
        start = time.time()
        world.simulation_start_time = start
        sim.update(dt)
        end = time.time()
        
        if (end - start) < dt:
            await asyncio.sleep(dt - (end - start))
                    
        if len(sim.agents_escaped) == num_agents:
            break
        await asyncio.sleep(0)
        
    if visualizer is not None:
        visualizer.play = False
    
    print("Simulation ended: " + str(len(sim.agents_escaped)) + " agents escaped in " + str(world.simulation_time) + " seconds.")
    
    return world.simulation_time, config.num_agents - len(sim.agents_escaped)
            
        
async def initialize_main():
    
    seed = config.random_seed
    if seed is not None:
        np.random.seed(seed)
    
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
        return await main_program(env, config)

if __name__ == "__main__":
    asyncio.run(initialize_main())
    