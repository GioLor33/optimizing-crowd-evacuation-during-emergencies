import asyncio
from visualization.visualizer import Visualizer

async def visualization_loop(visualizer):
    while visualizer.window_is_open() and visualizer.on:
        visualizer.create_drawing()
        await asyncio.sleep(0)  # yield control

async def main_program(visualizer):
    loop = asyncio.get_running_loop()
    while True:
        # Main executions
        pass
        

async def main():
    visualizer = Visualizer(name="Crowd Simulation")
    await asyncio.gather(
        visualization_loop(visualizer),
        main_program(visualizer),
    )

if __name__ == "__main__":
    asyncio.run(main())
