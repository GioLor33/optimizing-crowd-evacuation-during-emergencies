from environments.environment import Environment
import numpy as np
from aco_algorithm.node import Node

class GridGraph():
    def __init__(self, env_instance: Environment, n: int = None, m: int = None, k: int = 1):
        '''
        Docstring for __init__
        
        :param env_instance: 
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
        
        self.nodes = {}
        self.N = None
        self.nodes_id_set = set()
        self.exit_nodes = set()
        self.create_graph(env_instance, n, m, k)
        print("Grid graph created with " + str(len(self.nodes)) + " nodes.")
        
    def compute_node_id(self, i: int, j: int, n: int, m:int) -> int:
        i = max(0,i)
        i = min(i,n-1)
        
        j = max(0,j)
        j = min(j,m-1)
        
        return i * (n-1) + j
    
    def create_graph(self, env_instance: Environment, n: int, m: int, k: int):
        
        assert k in [1, 2], "k must be 1 (4-connectivity) or 2 (8-connectivity)"
        
        self.nodes = dict()

        # Add nodes
        env_width, env_length = self.env.get_dimensions()
        width_unit = env_width / m
        height_unit = env_length / n
        #count = 0
        for i in range(n): # number of row
            for j in range(m): # number of column
                pos = (width_unit / 2 + j * width_unit, height_unit / 2 + i * height_unit)
                if self.env.check_is_position_free(pos):
                    id = self.compute_node_id(i, j, n, m)
                    self.nodes[id] = Node(id, pos)
                    self.nodes_id_set.add(id)
                    # Like this, not all ids are used, but this enumeration is better to create edges
                    #self.nodes.add(Node(count, pos))
                    #count += 1
                    
        # Create edges between nodes
        nodes_set = set(self.nodes.keys())
        for i in range(n):
            for j in range(m):
                neighbors = [
                    self.compute_node_id(i, j+1, n, m),   # right
                    self.compute_node_id(i, j-1, n, m),   # left
                    self.compute_node_id(i-1, j, n, m),   # up
                    self.compute_node_id(i+1, j, n, m)    # down
                ]
                if k == 2:
                    neighbors += [
                        self.compute_node_id(i-1, j+1, n, m),   # up-right
                        self.compute_node_id(i-1, j-1, n, m),   # up-left
                        self.compute_node_id(i+1, j+1, n, m),   # down-right
                        self.compute_node_id(i+1, j-1, n, m)    # down-left
                    ]
                for neighbor_id in neighbors:
                    node_id = self.compute_node_id(i, j, n, m)
                    if neighbor_id in nodes_set and neighbor_id != node_id:
                        node = self.nodes[node_id]
                        neighbor_node = self.nodes[neighbor_id]
                        if self.env.check_something_reached((node.pos[0], node.pos[1]), (neighbor_node.pos[0], neighbor_node.pos[1]), "wall") is None:
                            dist = np.linalg.norm(np.array(node.pos) - np.array(neighbor_node.pos))
                            print(f"Adding edge between node {node_id} and node {neighbor_id} with distance {dist:.2f}.")
                            node.edges[neighbor_id] = dist 
                            neighbor_node.edges[node_id] = dist

        # Add exit nodes
        exits = env_instance.get_safety_exits()
        self.N = self.compute_node_id(n-1,m-1,n,m)
        self.N += 1
        for id, exit_points in enumerate(exits):
            exit_points = np.array(exit_points)
            mid = (exit_points[0] + exit_points[1]) / 2
            new_node = Node(self.N, mid)
            print(f"Adding exit node {new_node.id} at position {self.N}.")
            print(self.nodes.keys())
            self.nodes[self.N] = new_node
            print(self.nodes[131].pos)
            print(self.nodes.keys())
            self.exit_nodes.add(new_node.id)
            self.N += 1
            # count += 1
            
            # Add edges, only if distance is less than a threshold
            threshold = 3.0 # meters
            for node_id, node in self.nodes.items():
                if node_id == new_node.id:
                    continue
                dist = np.linalg.norm(np.array(node.pos) - mid)
                if dist <= threshold:
                    if self.env.check_something_reached((node.pos[0], node.pos[1]), (new_node.pos[0], new_node.pos[1]), "wall") is None:
                        node.edges[new_node.id] = dist
                        new_node.edges[node.id] = dist                     
    
    def run_aco(self, n_iterations=100, alpha=2.0, beta=1.0, evaporation_rate=0.7, Q=100):
        pheromone = np.ones((self.N, self.N))
                
        for iteration in range(n_iterations):
            all_paths = []
            all_path_lengths = []
                        
            for agent in self.env.agents:
                # print(f"\nAgent {agent.id}:")
                start_pos = agent.pos
                # Find the closest node to the agent's position
                # TODO: this could be improved by dividing the enviroment in a grid and storing the closest node for each cell
                # Maybe this implementation will not be precise if the agent is very close to the border of the grid cell, but still
                
                #This because otherwise at each iteration the starting point would be recomputed, but the result would be the same always
                if agent.start_node_id is None:
                    sorted_node_ids = sorted(
                        self.nodes.keys(),
                        key=lambda i: np.linalg.norm(np.array(self.nodes[i].pos) - np.array(start_pos))
                    )
                    while len(sorted_node_ids) > 0:
                        start_node_id = sorted_node_ids.pop(0)
                        if self.env.check_something_reached((start_pos[0], start_pos[1]), (self.nodes[start_node_id].pos[0], self.nodes[start_node_id].pos[1]), "wall") is None:
                            break
                    
                    #start_node_id = min(range(n_nodes), key=lambda i: np.linalg.norm(np.array(self.nodes[i].pos) - np.array(start_pos)))
                    agent.start_node_id = start_node_id
                
                path = [agent.start_node_id]
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
                
                # if path[-1] in self.exit_nodes and (path_length < agent.path_length or agent.path is None):
                #     agent.path = self.nodes_of(path)
                #     agent.path_length = path_length
            
            # Update pheromone
            pheromone *= (1 - evaporation_rate)
            for path, length in zip(all_paths, all_path_lengths):
                if path[-1] in self.exit_nodes:
                    for i in range(len(path) - 1):
                        pheromone[path[i]][path[i+1]] += Q / length
                        
        return pheromone
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i].pos) for i in path_indices]