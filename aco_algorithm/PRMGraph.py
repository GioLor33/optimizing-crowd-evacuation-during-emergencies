from environments.environment import Environment
import numpy as np
from scipy.spatial import KDTree

# IMPORTANT: per come è implementato ACO adesso, abbiamo che viene simulato per 100 iterazioni il movimento di ogni singolo agente
# e alla fine di queste 100 iterazioni viene scelto il percorso migliore per ogni agente (quindi non si basa sui singoli nodi, 
# ma sugli agenti)

class Node():
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.edges = {}  # dictionary with [key, value] as [neighbor_node.id, cost]

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
        self.exit_nodes = set()
        print("Creating PRM graph...")
        self.create_graph(env_instance, N, k)
        print("PRM graph created with " + str(len(self.nodes)) + " nodes.")
    
    def create_graph(self, env_instance: Environment, N: int, k: int):
        
        check_node_positions = set()
        self.nodes = []

        # Add random nodes
        for id in range(N):
            free = False
            while not free:
                pos = (np.random.uniform(0, env_instance.get_width()), np.random.uniform(0, env_instance.get_height()))
                free = env_instance.check_is_position_free(pos) and (pos not in check_node_positions)
            new_node = Node(id, pos)
            self.nodes.append(new_node)
            check_node_positions.add(pos)

        # Add exit nodes
        exits = env_instance.get_safety_exits()
        for id, exit_points in enumerate(exits):
            exit_points = np.array(exit_points)
            mid = (exit_points[0] + exit_points[1]) / 2
            new_node = Node(N + id, mid)
            self.nodes.append(new_node)
            self.exit_nodes.add(N + id)

        # Build KDTree
        nodes_list = self.nodes
        tree = KDTree(np.array([node.pos for node in nodes_list]))

        # Connect nodes
        for i, p1 in enumerate(nodes_list):
            point_coords = p1.pos
            dists, idxs = tree.query(point_coords, k=k+1)
            for dist, j in zip(dists[1:], idxs[1:]):  # skip itself
                p2 = nodes_list[j]

                if env_instance.check_something_reached((p1.pos[0], p1.pos[1]), (p2.pos[0], p2.pos[1]), "wall") is None:
                    exit_idx = env_instance.check_something_reached((p1.pos[0], p1.pos[1]), (p2.pos[0], p2.pos[1]), "exit")
                    if exit_idx is not None:
                        p2 = nodes_list[len(nodes_list) - len(exits) + exit_idx]

                    cost = np.linalg.norm([p1.pos[0] - p2.pos[0], p1.pos[1] - p2.pos[1]])
                    if p2 not in p1.edges:
                        p1.edges[p2.id] = cost
                    if p1 not in p2.edges:
                        p2.edges[p1.id] = cost
        
                        
    def run_aco(self, n_iterations=100, alpha=2.0, beta=1.0, evaporation_rate=0.7, Q=100):
        n_nodes = len(self.nodes)
        pheromone = np.ones((n_nodes, n_nodes))
                
        for iteration in range(n_iterations):
            all_paths = []
            all_path_lengths = []
                        
            for agent in self.env.agents:
                # print(f"\nAgent {agent.id}:")
                start_pos = agent.pos
                # Find the closest node to the agent's position
                # TODO: this could be improved by dividing the enviroment in a grid and storing the closest node for each cell
                # Maybe this implementation will not be precise if the agent is very close to the border of the grid cell, but still
                
                #This if beause otherwise at each iteration the starting point would be recomputed, but the result would be the same always
                if agent.start_node_id is not None:
                    start_node_id = agent.start_node_id
                else:
                    start_node_id = min(range(n_nodes), key=lambda i: np.linalg.norm(np.array(self.nodes[i].pos) - np.array(start_pos)))
                    agent.start_node_id = start_node_id
                
                path = [start_node_id]
                visited_id = set(path)
                
                while True:
                    current_node_id = path[-1]
                    if current_node_id in self.exit_nodes: # reached an exit node
                        # here we should check if the exit corresponds to the agent's target
                        # otherwise we should penalize the visit of this node !! (we do not want the agent to go out from the wrong exit)
                        # print(f"> Agent {agent.id} reached exit node {current_node_id} during simulation {iteration}.")
                        break
                    
                    neighbors = self.nodes[current_node_id].edges.items()
                    probabilities = []
                    for neighbor_id, cost in neighbors:
                        if neighbor_id not in visited_id:
                            tau = pheromone[current_node_id][neighbor_id] ** alpha
                            eta = 1 #(1.0 / cost) ** beta
                            probabilities.append(tau * eta)
                        else:
                            probabilities.append(0)
                                                
                    total = sum(probabilities)
                    if total == 0: # no unvisited_id neighbors
                        break
                    
                    probabilities = [p / total for p in probabilities]
                    next_node = np.random.choice([n for n, _ in neighbors], p=probabilities)
                    
                    path.append(next_node)
                    visited_id.add(next_node)
                    
                path_length = sum(np.linalg.norm(np.array(self.nodes[path[i]].pos) - np.array(self.nodes[path[i+1]].pos)) for i in range(len(path)-1))
                all_paths.append(path)
                all_path_lengths.append(path_length)
                
                if path[-1] in self.exit_nodes and (path_length < agent.path_length or agent.path is None):
                    agent.path = self.nodes_of(path)
                    agent.path_length = path_length
            
            # Update pheromone
            pheromone *= (1 - evaporation_rate)
            for path, length in zip(all_paths, all_path_lengths):
                if path[-1] in self.exit_nodes:
                    for i in range(len(path) - 1):
                        pheromone[path[i]][path[i+1]] += Q / length
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i].pos) for i in path_indices]