from boids_algorithm.boidsAgent import BoidsAgent

class CrowdSimulator:
    def __init__(self, environment_input, num_agents=200):
        self.env = environment_input
        self.env.set_agents([BoidsAgent(self.env, uid=i) for i in range(num_agents)])
        self.agents_escaped = []

    def update(self):
        active_agents_data = []
        survivors = []
        snapshot = list(self.env.agents)

        N = len(self.env.agents)
        for i in range(N-1, -1, -1):
            agent = self.env.agents[i]
            agent.update(snapshot)
            if self.env.check_something_reached(agent.prev_pos, agent.pos, "exit") is not None:
                self.agents_escaped.append(agent)
                self.env.agents.pop(i)
                print("Agent removed. Agents left in env: " + str(len(self.env.agents)))
            # else:
            #     survivors.append(agent)
            #     active_agents_data.append({
            #         'id': agent.id,
            #         'pos': (float(agent.pos[0]), float(agent.pos[1])),
            #         'vel': (float(agent.vel[0]), float(agent.vel[1]))
            #     })

        return self.agents_escaped