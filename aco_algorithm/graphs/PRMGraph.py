from environments.environment import Environment
from aco_algorithm.graphs.basicGraph import BasicGraph
from aco_algorithm.graphs.node import Node
from parser.config import Config
import numpy as np
from scipy.spatial import KDTree

# IMPORTANT: per come è implementato ACO adesso, abbiamo che viene simulato per 100 iterazioni il movimento di ogni singolo agente
# e alla fine di queste 100 iterazioni viene scelto il percorso migliore per ogni agente (quindi non si basa sui singoli nodi, 
# ma sugli agenti)

class PRMGraph(BasicGraph):
    def __init__(self, env_instance: Environment, config : Config):
        '''
        Docstring for __init__
        
        :param env_instance: 
        :type env_instance: Environment
        
        :param N: number of nodes
        :type N: int
        
        :param k: number of nearest neighbors to connect
        :type k: int
        '''
        
        super().__init__(env_instance, config.n, None, config.k_connectivity)
        self.create_graph()
        print("PRM graph created with " + str(len(self.nodes)) + " nodes.")
    
    def create_graph(self):
        
        check_node_positions = set()
        self.nodes = dict()

        # Add random nodes
        for id in range(self.n):
            free = False
            while not free:
                pos = (np.random.uniform(self.border, self.env.get_width()-self.border), np.random.uniform(self.border, self.env.get_height()-self.border))
                free = self.env.check_is_position_free(pos) and (pos not in check_node_positions)
            self.nodes[id] = Node(id, pos)
            self.nodes_id_set.add(id)
            check_node_positions.add(pos)
        self.N = len(self.nodes)

        # Add exit nodes
        exits = self.env.get_safety_exits()
        for id, exit_points in enumerate(exits):
            exit_points = np.array(exit_points)
            mid = (exit_points[0] + exit_points[1]) / 2
            self.nodes[self.N + id] = Node(self.N + id, mid)
            self.exit_nodes.add(self.N + id)
            self.N += 1

        # Build KDTree
        nodes_list = list(self.nodes.values())
        tree = KDTree(np.array([node.pos for node in nodes_list]))

        # Connect nodes
        for i, p1 in enumerate(nodes_list):
            point_coords = p1.pos
            dists, idxs = tree.query(point_coords, k=self.k+1)
            for dist, j in zip(dists[1:], idxs[1:]):  # skip itself
                p2 = nodes_list[j]

                if self.env.check_something_reached((p1.pos[0], p1.pos[1]), (p2.pos[0], p2.pos[1]), "wall") is None:
                    exit_idx = self.env.check_something_reached((p1.pos[0], p1.pos[1]), (p2.pos[0], p2.pos[1]), "exit")
                    if exit_idx is not None:
                        p2 = nodes_list[len(nodes_list) - len(exits) + exit_idx]

                    cost = np.linalg.norm([p1.pos[0] - p2.pos[0], p1.pos[1] - p2.pos[1]])
                    if p2 not in p1.edges:
                        p1.edges[p2.id] = cost
                    if p1 not in p2.edges:
                        p2.edges[p1.id] = cost
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i].pos) for i in path_indices]