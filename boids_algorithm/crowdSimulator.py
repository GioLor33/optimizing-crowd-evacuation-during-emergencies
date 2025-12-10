import numpy as np
from boids_algorithm.boidsAgent import BoidsAgent


class CrowdSimulator:
    def __init__(self, world, config):
        self.world = world
        self.config = config
        self.agents_escaped = []
        if len(self.world.agents) == 0:
            spawn_positions = []
            if hasattr(self.world, "get_ordered_spawn_positions"):
                spawn_positions = self.world.get_ordered_spawn_positions(config.num_agents)

            for i in range(config.num_agents):
                start_pos = spawn_positions[i] if i < len(spawn_positions) else None
                agent = BoidsAgent(self.world, self.config, start_pos=start_pos)
                self.world.add_agent(agent)

    def update(self, dt):
        """
        Updates the simulation one step.
        Iterates backwards or over a copy to safely remove agents.
        """
        for agent in self.world.agents[:]:

            # 1. Physics Update
            agent.update(dt)

            # 2. EXIT CHECK 1: Intersection
            reached_object = self.world.check_something_reached(
                agent.prev_pos,
                agent.pos,
                "exit"
            )

            if reached_object is not None:
                self.remove_agent(agent)
                continue

                # 3. EXIT CHECK 2: Proximity (Anti-Pile-Up)
            target = agent.get_smart_target()
            dist = np.linalg.norm(agent.pos - target)

            if dist < 0.5:
                self.remove_agent(agent)

    def remove_agent(self, agent):
        if agent in self.world.agents:
            self.world.agents.remove(agent)
            self.agents_escaped.append(agent)