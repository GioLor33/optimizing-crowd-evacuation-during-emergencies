from environments.environment import Environment
from aco_algorithm.graphs.basicGraph import BasicGraph
from aco_algorithm.graphs.node import Node
from parser.config import Config
import numpy as np

class GridGraph(BasicGraph):
    def __init__(self, env_instance: Environment, config : Config):
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
        
        super().__init__(env_instance, config.n, config.m, config.k_connectivity)
        assert config.k_connectivity in [1, 2], "k must be 1 (4-connectivity) or 2 (8-connectivity)"
        self.create_graph()
        
    def compute_node_id(self, i: int, j: int) -> int:
        if i < 0 or i >= self.n or j < 0 or j >= self.m:
            return -1
        return i * self.m + j
    
    def create_graph(self):
        
        self.nodes = dict()

        # Add nodes
        env_width, env_length = self.env.get_dimensions()
        width_unit = (env_width - self.border*2) / self.m     # -> let's leave a margin of 0.5 meter from each side 
        height_unit = (env_length - self.border*2) / self.n   # -> let's leave a margin of 0.5 meter from each side
        
        for i in range(self.n): # number of row
            for j in range(self.m): # number of column
                pos = (self.border + width_unit / 2 + j * width_unit, self.border + height_unit / 2 + i * height_unit)
                if self.env.check_is_position_free(pos):
                    id = self.compute_node_id(i, j)
                    self.nodes[id] = Node(id, pos)
                    self.nodes_id_set.add(id)
                    # Like this, not all ids are used, but this enumeration is better to create edges

        # Create edges between nodes
        nodes_set = set(self.nodes.keys())
        for i in range(self.n):
            for j in range(self.m):
                neighbors = [
                    self.compute_node_id(i, j+1),   # right
                    self.compute_node_id(i, j-1),   # left
                    self.compute_node_id(i-1, j),   # up
                    self.compute_node_id(i+1, j)    # down
                ]
                if self.k == 2:
                    neighbors += [
                        self.compute_node_id(i-1, j+1),   # up-right
                        self.compute_node_id(i-1, j-1),   # up-left
                        self.compute_node_id(i+1, j+1),   # down-right
                        self.compute_node_id(i+1, j-1)    # down-left
                    ]
                node_id = self.compute_node_id(i, j)
                node = self.nodes[node_id]
                for neighbor_id in neighbors:
                    if neighbor_id < 0:
                        continue
                    if neighbor_id in nodes_set and neighbor_id != node_id:
                        neighbor_node = self.nodes[neighbor_id]
                        if self.env.check_something_reached((node.pos[0], node.pos[1]), (neighbor_node.pos[0], neighbor_node.pos[1]), "wall") is None:
                            dist = np.linalg.norm(np.array(node.pos) - np.array(neighbor_node.pos))
                            node.edges[neighbor_id] = dist 
                            neighbor_node.edges[node_id] = dist

        # Add exit nodes
        exits = self.env.get_safety_exits()
        self.N = self.compute_node_id(self.n-1,self.m-1)
        self.N += 1
        for id, exit_points in enumerate(exits):
            exit_points = np.array(exit_points)
            l = np.linalg.norm(exit_points[1] - exit_points[0])
            
            e = max(int(l / 0.5), 1) # we will add a node every 0.5 meter, such that it may happen that multiple people can pass through the door at the same time, hence multiple nodes are added to the graph
            margin = (l - (e - 1) * 0.5) / 2
            for k in range(e):
                
                new_exit_node_pos = exit_points[0] + (k / e) * (exit_points[1] - exit_points[0]) + (margin) * (exit_points[1] - exit_points[0]) / l
                new_node = Node(self.N, new_exit_node_pos)
                self.nodes[self.N] = new_node
                self.exit_nodes.add(new_node.id)
                self.N += 1
            
                # Add edges to the exit node only if distance is less than a threshold
                threshold = 3.0 # meters
                for node_id, node in self.nodes.items():
                    if node_id == new_node.id:
                        continue
                    dist = np.linalg.norm(np.array(node.pos) - new_exit_node_pos)
                    if dist <= threshold:
                        if self.env.check_something_reached((node.pos[0], node.pos[1]), (new_node.pos[0], new_node.pos[1]), "wall") is None:
                            node.edges[new_node.id] = dist
                            new_node.edges[node.id] = dist                  
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i].pos) for i in path_indices]