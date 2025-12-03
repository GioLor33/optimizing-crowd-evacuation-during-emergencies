import asyncio
from visualization.visualizer import Visualizer
from environments.environment import Environment

async def visualization_loop(visualizer):
    assert isinstance(visualizer, Visualizer)
    while visualizer.window_is_open() and visualizer.on:
        visualizer.create_drawing()
        await asyncio.sleep(0)  # yield control

async def main_program(visualizer, environment):
    assert isinstance(visualizer, Visualizer)
    loop = asyncio.get_running_loop()
    #print("Current Environment Grid:")
    #print(environment)
    
    while True:
        # Main executions
        await asyncio.sleep(0)
        
        
async def initialize_main():
    
    env = Environment(
        name="Test Environment",
        dimensions=(10, 10),
        walls=[[(1, 1), (1, 2)]],
        exits=[[(0, 0), (10, 0)]]
    )
        
    visualizer = Visualizer(environment=env, name="Crowd Simulation")
    await asyncio.gather(
        visualization_loop(visualizer),
        main_program(visualizer, env),
    )

if __name__ == "__main__":
    asyncio.run(initialize_main())
