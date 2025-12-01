from environments.environment import Environment
import numpy as np
from scipy.spatial import KDTree

class PRMGraph():
    def __init__(self, env_instance: Environment, N: int, k:int):
        '''
        Docstring for __init__
        
        :param env_instance: 
        :type env_instance: Environment
        
        :param N: number of nodes
        :type N: int
        
        :param k: number of nearest neighbors to connect
        :type k: int
        '''
        
        self.env = env_instance
        
        self.nodes = []
        self.edges = {} # graph stored as an adjacency list
        self.create_graph(env_instance, N, k)
    
    def create_graph(self, env_instance: Environment, N: int, k: int):
        
        for _ in range(N):
            free = False
            while not free:
                x = np.random.uniform(0, env_instance.get_width())
                y = np.random.uniform(0, env_instance.get_height())
                free = env_instance.check_is_position_free((x,y))
            self.nodes.append((x, y))
            
        exits = env_instance.get_safety_exits()
        for exit_points in exits:
            exit_points = np.array(exit_points)
            mid = ((exit_points[0] + exit_points[1]) * 1.0 / 2)
            self.nodes.append(mid)
            
        self.edges = {i: [] for i in range(len(self.nodes))}
        
        tree = KDTree(np.array(self.nodes))
        for i, p in enumerate(self.nodes):
            dists, idxs = tree.query(p, k=k+1)
            for dist, j in zip(dists[1:], idxs[1:]):  # skip itself
                p1 = self.nodes[i]
                p2 = self.nodes[j]
                if env_instance.check_something_reached(p1, p2, "wall") is None:
                    exit = env_instance.check_something_reached(p1, p2, "exit")
                    if exit is None:
                        self.edges[i].append((j, dist))  # store neighbor and edge cost
                    else:
                        exit_node_index = len(self.nodes) - len(env_instance.get_safety_exits()) + exit
                        # mid = ((exit[0] + exit[1]) / 2)
                        self.edges[i].append((exit_node_index, dist))
                        
    def run_aco(self, n_iterations=100, alpha=1.0, beta=2.0, evaporation_rate=0.5, Q=100):
        n_safety_exits = len(self.env.get_safety_exits())
        n_points = len(self.nodes)
        pheromone = np.ones((n_points, n_points))
        
        for iteration in range(n_iterations):
            all_paths = []
            all_path_lengths = []
            
            for agent in self.env.agents:
                start_pos = agent.pos
                # Find the closest node to the agent's position
                start_node = min(range(n_points), key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(start_pos)))
                
                path = [start_node]
                visited = set(path)
                
                while True:
                    current_node = path[-1]
                    if current_node >= n_points - n_safety_exits: # reached an exit node
                        break
                    
                    neighbors = self.edges[current_node]
                    probabilities = []
                    for neighbor, cost in neighbors:
                        if neighbor not in visited:
                            tau = pheromone[current_node][neighbor] ** alpha
                            eta = (1.0 / cost) ** beta
                            probabilities.append(tau * eta)
                        else:
                            probabilities.append(0)
                    
                    total = sum(probabilities)
                    if total == 0: # no unvisited neighbors
                        break
                    
                    probabilities = [p / total for p in probabilities]
                    next_node = np.random.choice([n for n, _ in neighbors], p=probabilities)
                    
                    path.append(next_node)
                    visited.add(next_node)
                
                path_length = sum(np.linalg.norm(np.array(self.nodes[path[i]]) - np.array(self.nodes[path[i+1]])) for i in range(len(path)-1))
                all_paths.append(path)
                all_path_lengths.append(path_length)
                
                if path_length < agent.path_length or agent.path is None:
                    agent.path = self.nodes_of(path)
                    agent.path_length = path_length
            
            # Update pheromone
            pheromone *= (1 - evaporation_rate)
            for path, length in zip(all_paths, all_path_lengths):
                for i in range(len(path) - 1):
                    pheromone[path[i]][path[i+1]] += Q / length
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i]) for i in path_indices]