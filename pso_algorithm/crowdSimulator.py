from pso_algorithm.psoAgent import LocalPSOBoidsAgent
from pso_algorithm.psoConfig import PSOConfig
from pso_algorithm.psoAgent import GridFitness
from parser.config import Config

class CrowdSimulator:
    def __init__(self, environment_input, config: Config):
        self.config = PSOConfig(config.file_path)
        self.env = environment_input
        self.agents_escaped = []

        num_agents = self.config.num_agents

        fitness_map = GridFitness(self.env)

        # Create PSO Boids agents
        self.env.set_agents([
            LocalPSOBoidsAgent(
                env_instance=self.env,
                uid=i,
                config=self.config,
                fitness_map=fitness_map
            )
            for i in range(num_agents)
        ])

    def update(self, dt):
        snapshot = list(self.env.agents)
        N = len(self.env.agents)

        for i in range(N - 1, -1, -1):
            agent = self.env.agents[i]
            agent.update(snapshot, dt)

            hit_exit = self.env.check_something_reached(agent.prev_pos, agent.pos, "exit")
            x, y = agent.pos
            w, h = self.env.get_dimensions()
            is_out_of_bounds = (x >= w) or (x <= 0) or (y >= h) or (y <= 0)

            if hit_exit is not None or is_out_of_bounds:
                self.agents_escaped.append(agent)
                self.env.agents.pop(i)
                print(f"Agent removed. Agents left: {len(self.env.agents)}")

        self.env.simulation_time += dt
        return self.agents_escaped
