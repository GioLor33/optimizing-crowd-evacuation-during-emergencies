from environments.environment import Environment
from aco_algorithm.graphs.node import Node
import numpy as np
import time

class BasicGraph():

    def __init__(self, env_instance: Environment, n: int = None, m = None, k: int = 1):
        '''
        Docstring for __init__
        
        :param env_instance: environment instance
        :type env_instance: Environment
        
        :param n: number of nodes along the height of the environment (rows)
        :type n: int
        
        :param m: number of nodes along the width of the environment (cols)
        :type m: int
        
        :param k: type of neighbors to consider, where 1=4-connectivity and 2=8-connectivity
        :type k: int
        '''
        
        self.env = env_instance
        self.n = n
        self.m = m
        self.k = k
        
        self.nodes = dict()
        self.N = None
        self.nodes_id_set = set()
        self.exit_nodes = set()
        
        self.num_ants = None
        self.num_iterations = None
        self.evaporation_rate = None
        self.alpha = None  # Importance of pheromone
        self.beta = None    # Importance of heuristic information
        self.pheromone = dict()  # key: (node1_id, node2_id), value: pheromone level
        
        self.border = 0.5  # margin from the environment borders to place nodes
        
    def initialize_aco_parameters(self, num_ants, num_iterations, evaporation_rate, alpha, beta):
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.evaporation_rate = evaporation_rate
        self.alpha = alpha  
        self.beta = beta    
        
    def run_aco(self):
        self.initialize_pheromones()
        
        for iteration in range(self.num_iterations):
            ants_pos = self.initialize_ants_positions() # here we place the ants in a random position on the graph in a uniform way
            all_paths = []
            all_path_lengths = []
            
            #print(f"ACO iteration {iteration+1}/{self.num_iterations}...")
                        
            for agent in range(self.num_ants):
                start_node_id = ants_pos[agent]
                
                path = [start_node_id]
                visited_id = set(path)
                
                create_path = False
                while True:
                    current_node_id = path[-1]
                    if current_node_id in self.exit_nodes: # reached an exit node
                        # here we should check if the exit corresponds to the agent's target
                        # otherwise we should penalize the visit of this node !! (we do not want the agent to go out from the wrong exit)
                        # print(f"> Agent {agent.id} reached exit node {current_node_id} during simulation {iteration}.")
                        create_path = True
                        break
                    
                    neighbors = list(self.nodes[current_node_id].edges.items())
                    probabilities = []
                    for neighbor_id, cost in neighbors:
                        if neighbor_id not in visited_id:
                            tau = self.pheromone[(current_node_id, neighbor_id)] ** self.alpha
                            eta = (1 / cost) ** self.beta
                            probabilities.append(tau * eta)
                        else:
                            probabilities.append(0)
                                                
                    total = sum(probabilities)
                    if total == 0: # no unvisited_id neighbors
                        create_path = False
                        break
                    
                    probabilities = [p / total for p in probabilities]
                    next_node = np.random.choice([n for n, _ in neighbors], p=probabilities)
                    
                    path.append(next_node)
                    visited_id.add(next_node)
                
                if create_path:
                    path_length = sum(np.linalg.norm(np.array(self.nodes[path[i]].pos) - np.array(self.nodes[path[i+1]].pos)) for i in range(len(path)-1))
                    all_paths.append(path)
                    all_path_lengths.append(path_length)
            
            # Update pheromone
            for k in self.pheromone:
                self.pheromone[k] *= (1 - self.evaporation_rate)
            for path, length in zip(all_paths, all_path_lengths):
                if path[-1] in self.exit_nodes:
                    for i in range(len(path) - 1):
                        self.pheromone[(path[i], path[i+1])] += 1 / length
                        
        return self.pheromone

    def initialize_pheromones(self, initial_pheromone = 1.0):
        for i in self.nodes:
            for j in self.nodes[i].edges.keys():
                self.pheromone[(i, j)] = initial_pheromone
        
    def initialize_ants_positions(self):
        ants_pos = []
        for _ in range(self.num_ants):
            ants_pos.append(np.random.choice(list(self.nodes.keys())))
        return ants_pos
                