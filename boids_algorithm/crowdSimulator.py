import numpy as np
from boids_algorithm.boidsAgent import BoidsAgent

class CrowdSimulator:
    def __init__(self, world, config):
        self.world = world
        self.config = config

        self.agents_escaped = []

        # # Initialize agents if they haven't been added yet
        # if len(self.world.agents) == 0:
        #     spawn_positions = []
        #     if hasattr(self.world, "get_ordered_spawn_positions"):
        #         spawn_positions = self.world.get_ordered_spawn_positions(config.num_agents)

        #     for i in range(config.num_agents):
        #         start_pos = spawn_positions[i] if i < len(spawn_positions) else None
        #         # Pass the main config to the agent
        #         agent = BoidsAgent(self.world, self.config, start_pos=start_pos)
        #         self.world.add_agent(agent)

    def update(self, dt):
        """
        Updates the simulation one step.
        Iterates over a copy of the list to safely remove agents.
        """
        agents_snapshot = list(self.world.agents)

        for agent in agents_snapshot:
            agent.update(dt, agents_snapshot)

            # 1. EXIT CHECK: Intersection
            reached_object = self.world.check_something_reached(
                agent.prev_pos,
                agent.pos,
                "exit"
            )

            if reached_object is not None:
                self.remove_agent(agent)

        self.world.simulation_time += dt

    def remove_agent(self, agent):
        if agent in self.world.agents:
            agent.safe = True
            self.world.agents.remove(agent)
            self.agents_escaped.append(agent)