from aco_algorithm.acoAgent import AcoAgent
from environments.utils import path_intersection_in_time, segments_intersect_new
import numpy as np
from parser.config import Config

class CrowdSimulator():
    def __init__(self, environment_input, config:Config):
        self.config = config
        
        self.env = environment_input
        num_agents = self.config.num_agents
        #self.env.set_agents([AcoAgent(self.env, uid=i) for i in range(num_agents)])
        print(f"Adding {num_agents} agents to the env")
        for i in range(num_agents):
            self.env.add_agent(AcoAgent(self.env, uid=i))
        self.agents_escaped = []
        
        if config.graph_type == "grid":
            from aco_algorithm.graphs.gridGraph import GridGraph
            self.aco_env = GridGraph(self.env, self.config)
            
        elif config.graph_type == "PRM":
            from aco_algorithm.graphs.PRMGraph import PRMGraph
            # # TODO: add parameters N and k in config file
            # dims = self.env.get_dimensions()
            # N = max(dims[0] * dims[1] // 2, 10)
            # k = max(min(N,5), N // 10)
            self.aco_env = PRMGraph(self.env, self.config)
            
        else:
            raise ValueError("Graph type " + str(config.graph_type) + " not recognized.")
        
        print("Starting ACO algorithm...")
        self.aco_env.initialize_aco_parameters(
            num_ants=self.config.num_ants,
            num_iterations=self.config.num_iterations,
            alpha=self.config.alpha,
            beta=self.config.beta,
            evaporation_rate=self.config.evaporation_rate
        )
        self.aco_env.run_aco()
        
        # self.set_agents_next_target()
        self.set_agents_first_target()
        
    # def update(self, dt):
  
    #     N = len(self.env.agents)
        
    #     snapshot = list(self.env.agents)
    #     for agent in self.env.agents:
    #         # if agent.path is None:
    #         #     # TODO: handle this case better
    #         #     self.env.agents.remove(agent)
    #         #     continue
    #         prev_pos = agent.pos.copy()
    #         agent.update(snapshot, self.env, dt)
            
    #         # check if target place is reached
            
    #         if len(agent.path) == 1: # if needed to avoid computing check_something_reached() at every iteration
    #             if self.env.check_something_reached((prev_pos[0], prev_pos[1]), (agent.pos[0], agent.pos[1]), "exit") is not None:
    #                 agent.node_reached()
    #         elif np.linalg.norm(agent.pos - agent.target) < 1.0:
    #             agent.node_reached()
            
    #         if agent.safe:
    #             self.agents_escaped.append(agent.id)
    #             self.env.agents.remove(agent)
                
    #     self.env.simulation_time += dt

    #     return
    
    def update(self, dt):
  
        N = len(self.env.agents)
        
        snapshot = list(self.env.agents)
        for agent in self.env.agents:
            
            if agent.fail:
                continue
            
            prev_pos = agent.pos.copy()
            agent.update(snapshot, self.env, dt)
            
            extra = 0.1 * agent.vel / np.linalg.norm(agent.vel) # this extra is added because otherwise agents tend to stop on the exit due to the social-force model
            if self.env.check_something_reached((prev_pos[0], prev_pos[1]), (agent.pos[0] + extra[0], agent.pos[1] + extra[1]), "exit") is not None:
                self.agents_escaped.append(agent.id)
                self.env.agents.remove(agent)
                
            #if agent.target_id in self.aco_env.exit_nodes:
            elif agent.target_id in self.aco_env.exit_nodes:
                continue
                    
            # check if target place is reached
            elif np.linalg.norm(agent.pos - agent.target) < 1: # TODO: threshold should be adaptive wrt velocity of the agent 
                agent.node_visited.add(agent.target_id)
                
                max = -1
                idx = -1
                for neighbor_id in self.aco_env.nodes[agent.target_id].edges.keys():
                    if neighbor_id not in agent.node_visited and self.aco_env.pheromone[frozenset({agent.target_id, neighbor_id})] > max:
                        max = self.aco_env.pheromone[frozenset({agent.target_id, neighbor_id})]
                        idx = neighbor_id
                # max = np.argmax(self.pheromones[agent.target_id])
                # if idx != agent.target_id:
                #     agent.target_id = idx
                # else:
                #     new_target_node_id = np.random.choice(list(self.aco_env.nodes[agent.target_id].edges.keys()))
                #     agent.target_id = new_target_node_id
                #     print(new_target_node_id)
                if idx < 0:
                    agent.fail = True
                    print("No valid next target found for agent " + str(agent.id))
                    continue
                agent.target_id = idx
                agent.target = self.aco_env.nodes[agent.target_id].pos
         
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
    
    def set_agents_first_target(self):
        n_nodes = len(self.aco_env.nodes)
        for agent in self.env.agents:
            # find closest node on the graph to go to, where the straight line from the current position to the node position is not intercepted by a wall
            
            # if isinstance(self.aco_env, PRMGraph):
            #     sorted_node_ids = sorted(
            #         range(n_nodes),
            #         key=lambda i: np.linalg.norm(
            #             np.array(self.aco_env.nodes[i].pos) - np.array(agent.pos)
            #         )
            #     )
            # elif isinstance(self.aco_env, GridGraph):
            sorted_node_ids = sorted(
                self.aco_env.nodes_id_set,
                key=lambda node_id: np.linalg.norm(
                    np.array(self.aco_env.nodes[node_id].pos) - np.array(agent.pos)
                )
            )
            # else:
            #     raise ValueError("Graph type not recognized.")
                
            start_node_id = None
            while len(sorted_node_ids) > 0:
                start_node_id = sorted_node_ids.pop(0)
                if self.env.check_something_reached((agent.pos[0], agent.pos[1]), (self.aco_env.nodes[start_node_id].pos[0], self.aco_env.nodes[start_node_id].pos[1]), "wall") is None:
                    break
            if start_node_id is None:
                raise ValueError("No valid start node found for agent " + str(agent.id))
            
            # start_node_id = min(range(n_nodes), key=lambda i: np.linalg.norm(np.array(self.nodes[i].pos) - np.array(start_pos)))
            agent.target_id = start_node_id
            agent.target = self.aco_env.nodes[start_node_id].pos
            
    
    # def set_agents_next_target(self):
    #     for agent in self.env.agents:
    #         if agent.path is not None:
    #             agent.target = agent.path[0]
    #     return
    
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
                