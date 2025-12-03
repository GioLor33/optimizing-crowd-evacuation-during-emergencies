from aco_algorithm.PRMGraph import PRMGraph
from aco_algorithm.acoAgent import AcoAgent

class CrowdSimulator():
    def __init__(self, environment_input, config): #, config):
        self.config = config
        
        self.env = environment_input
        num_agents = self.config.num_agents
        self.env.set_agents([AcoAgent(self.env, uid=i) for i in range(num_agents)])
        self.agents_escaped = []
        
        N = max(self.config.world_dimensions[0] * self.config.world_dimensions[1] // 10, 10)
        k = max(min(N,5), N // 10)
        self.aco_env = PRMGraph(self.env, N=N, k=k)
        self.paths_for_agents = self.aco_env.run_aco()

    def update(self, dt):
  
        N = len(self.env.agents)
        snapshot = list(self.env.agents)
        
        for i in range(N-1, -1, -1):
            agent = self.env.agents[i]
            if agent.path is None:
                continue
            agent.update(snapshot, dt)
            if agent.safe:
                self.agents_escaped.append(agent)
                self.env.agents.pop(i)
                print(f"Agent removed. Left: {len(self.env.agents)}")
                
        for i in range(N-1, -1, -1):
            agent.future_pos = None
                
        self.env.simulation_time += dt

        return self.agents_escaped