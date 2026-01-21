from pso_algorithm.psoAgent import LocalPSOAgent
from parser.config import Config
from pso_algorithm.psoAgent import GridFitness
import numpy as np

class CrowdSimulator:
    def __init__(self, environment_input, config: Config, fitness_map=None):
        self.config = config
        self.env = environment_input
        self.agents_escaped = []
        
        self.fitness_map = GridFitness(self.env)
        for agent in self.env.agents:
            if isinstance(agent, LocalPSOAgent):
                agent.initialize(config, self.fitness_map)

    def update(self, dt):
        snapshot = list(self.env.agents)
        N = len(self.env.agents)

        for i in range(N - 1, -1, -1):
            agent = self.env.agents[i]
            agent.update(snapshot, self.env, dt)

            prev_pos = agent.pos.copy()
            agent.update(snapshot, self.env, dt)
                        
            extra = 0.1 * agent.vel / np.linalg.norm(agent.vel) # this extra is added because otherwise agents tend to stop on the exit due to the social-force model
            if self.env.check_something_reached(prev_pos, (agent.pos[0] + extra[0], agent.pos[1] + extra[1]), "exit") is not None:
                self.agents_escaped.append(agent.id)
                self.env.agents.remove(agent)
            # se è uscito dai muri lo rimetto nella posizione precedente
            elif agent.pos[0] < 0:
                agent.pos[0] = 0.1
            elif agent.pos[1] < 0:
                agent.pos[1] = 0.1
            elif agent.pos[0] > self.env.get_dimensions()[0]: 
                agent.pos[0] = self.env.get_dimensions()[0] - 0.1
            elif agent.pos[1] > self.env.get_dimensions()[1]:
                agent.pos[1] = self.env.get_dimensions()[1] - 0.1

        self.env.simulation_time += dt
        return self.agents_escaped
