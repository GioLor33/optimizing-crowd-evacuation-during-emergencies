import numpy as np
import random
import asyncio
from visualization.visualizer import Visualizer
from environments.environment import Environment
from boids_algorithm.crowdSimulator import CrowdSimulator
import time

async def visualization_loop(visualizer):
    assert isinstance(visualizer, Visualizer)
    while visualizer.window_is_open() and visualizer.on:
        visualizer.create_drawing()
        await asyncio.sleep(0)  # yield control

async def main_program(visualizer, world):
    assert isinstance(visualizer, Visualizer)
    loop = asyncio.get_running_loop()
    
    num_agents = 10 # TODO: read this from config file
    dt = 0.05  # TODO: read this from config file
    sim = CrowdSimulator(world, num_agents=num_agents)
    while True:  # Main executions
        start = time.time()
        sim.update(dt)
        end = time.time()
        
        if (end - start) < dt:
            time.sleep(dt - (end - start))
            
        if len(sim.agents_escaped) == num_agents:
            break
        await asyncio.sleep(0)
    print("All agents have evacuated.")
    # TODO: display "Simulation Complete" message on visualizer
        
        
async def initialize_main():
    
    env = Environment(
        name="Test World",
        dimensions=(100, 100),
        exits=[[(100, 40), (100, 60)]],
        walls=[[(50, 0), (50, 70)]]
    )
    env.add_external_walls()
        
    visualizer = Visualizer(environment=env, name="Crowd Simulation")
    await asyncio.gather(
        visualization_loop(visualizer),
        main_program(visualizer, env),
    )

if __name__ == "__main__":
    asyncio.run(initialize_main())
    