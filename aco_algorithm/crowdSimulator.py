from aco_algorithm.PRMGraph import PRMGraph
from aco_algorithm.acoAgent import AcoAgent
from environments.utils import path_intersection_in_time, segments_intersect_new
import numpy as np
from parser.config import Config

class CrowdSimulator():
    def __init__(self, environment_input, config:Config):
        self.config = config
        
        self.env = environment_input
        num_agents = self.config.num_agents
        self.env.set_agents([AcoAgent(self.env, uid=i) for i in range(num_agents)])
        self.agents_escaped = []
        
        # TODO: add parameters N and k in config file
        N = max(self.config.world_dimensions[0] * self.config.world_dimensions[1] // 2, 10)
        N = 6
        k = max(min(N,5), N // 10)
        print(f"Creating PRM graph with N={N} nodes and k={k} neighbors.")
        self.aco_env = PRMGraph(self.env, N=N, k=k)
        self.paths_for_agents = self.aco_env.run_aco()
        self.set_agents_next_target()
        
    def update(self, dt):
  
        N = len(self.env.agents)
        
        snapshot = list(self.env.agents)
        for agent in self.env.agents:
            if agent.path is None:
                # TODO: handle this case better
                self.env.agents.remove(agent)
                continue
            prev_pos = agent.pos.copy()
            agent.update(snapshot, self.env, dt)
            
            # check if target place is reached
            if np.linalg.norm(agent.pos - agent.target) < 1:
                agent.node_reached()
            
            if agent.safe:
                self.agents_escaped.append(agent.id)
                self.env.agents.remove(agent)
                
        self.env.simulation_time += dt

        return

    def update_old(self, dt):
  
        N = len(self.env.agents)
        
        env_agents = set()
        for agent in self.env.agents:
            if agent.path is None:
                continue
            agent.compute_velocity(dt)
            env_agents.add(agent)
            
        for agent in self.env.agents:
            if agent.path is None:
                continue
            for agent2 in self.env.agents:
                if agent.id == agent2.id or agent2.path is None:
                    continue
                self.avoid_agents(agent, agent2, dt)
        
        # while len(env_agents) > 1: # if only one agent inside, it means it will not collide with anyone
        #     agent1 = env_agents.pop()
        #     agent2 = env_agents.pop()
        #     #env_agents.add(self.avoid_agents(agent1, agent2, dt))
        #     self.avoid_agents(agent1, agent2, dt)
            
        for agent in self.env.agents:
            if agent.path is None:
                continue
            agent.move(dt)
            if agent.safe:
                self.agents_escaped.append(agent.id)
                self.env.agents.remove(agent)
                
        self.env.simulation_time += dt

        return
    
    def set_agents_next_target(self):
        for agent in self.env.agents:
            if agent.path is not None:
                agent.target = agent.path[0]
                print(f"Agent {agent.id} target set to {agent.target}.")
        return
    
    def avoid_agents(self, agent1, agent2, dt):
        # !! In teoria future_pos non dovrebbe servire, controllare e nel caso eliminare !!
        if agent1.id == agent2.id:
            raise ValueError("Case which should not have happened, something is wrong.")
            return
        if agent1.future_pos is None or agent2.future_pos is None:
            raise ValueError("Future position of one of the agents is None, something is wrong.")
        
        if path_intersection_in_time(agent1.pos, agent1.vel, agent2.pos, agent2.vel, dt) is not None:
            
            n1 = np.linalg.norm(agent1.vel)
            n2 = np.linalg.norm(agent2.vel)
            if n1 == 0 or n2 == 0:
                return # one or both of the agents are not moving
            
            u1 = agent1.vel / n1
            u2 = agent2.vel / n2

            # Same direction → angle ≈ 0 → dot ≈ +1
            if np.dot(u1, u2) > 0.999:
                if n1 > n2:
                    agent1.vel = agent2.vel
                return
            else:
                agent1.vel = agent1.vel * 0.5
            # if self.vel[0] == 0 and self.vel[1] == 0:
            #     break
            #print(f"{self.vel}. The agent is in position {self.pos}, other agent in position {other_agent.pos}. Exiting the while loop")
                