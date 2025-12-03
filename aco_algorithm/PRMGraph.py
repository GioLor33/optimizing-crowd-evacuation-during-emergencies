from environments.environment import Environment
import numpy as np
from scipy.spatial import KDTree

# IMPORTANT: per come è implementato ACO adesso, abbiamo che viene simulato per 100 iterazioni il movimento di ogni singolo agente
# e alla fine di queste 100 iterazioni viene scelto il percorso migliore per ogni agente (quindi non si basa sui singoli nodi, 
# ma sugli agenti)

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
        self.exit_nodes = set()
        
        self.nodes = []
        self.edges = {} # graph stored as an adjacency list
        print("Creating PRM graph...")
        self.create_graph(env_instance, N, k)
        print("PRM graph created with " + str(len(self.nodes)) + " nodes.")
    
    def create_graph(self, env_instance: Environment, N: int, k: int):
        
        set_nodes = set()
        for _ in range(N):
            free = False
            while not free:
                x = np.random.uniform(0, env_instance.get_width())
                y = np.random.uniform(0, env_instance.get_height())
                free = env_instance.check_is_position_free((x,y)) and ((x, y) not in set_nodes)
            self.nodes.append((x, y))
            set_nodes.add((x, y))
        
        # # TO CHANGE
        # self.nodes.append((self.env.get_width() / 3, self.env.get_height() / 2))     # start from center
        # self.nodes.append((self.env.get_width() / 2, self.env.get_height() / 2))     # go straight
        # # self.nodes.append((self.env.get_width() / 2, self.env.get_height() / 3))
        # self.nodes.append((2 * self.env.get_width() / 3, self.env.get_height() / 3)) # deviation to test algorithm
        # k = 3
        # # END TO CHANGE
            
        exits = env_instance.get_safety_exits()
        for exit_points in exits:
            exit_points = np.array(exit_points)
            mid = ((exit_points[0] + exit_points[1]) * 1.0 / 2)
            self.nodes.append(mid)
            self.exit_nodes.add(len(self.nodes) - 1)
            
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
        print(self.edges)
                        
    def run_aco(self, n_iterations=100, alpha=2.0, beta=1.0, evaporation_rate=0.7, Q=100):
        n_safety_exits = self.env.get_safety_exits_number()
        n_points = len(self.nodes)
        pheromone = np.ones((n_points, n_points))
        
        # print(f"Nodes checked as safety exits: {self.exit_nodes}")
        
        for iteration in range(n_iterations):
            all_paths = []
            all_path_lengths = []
            
            # print(f"ACO iteration {iteration}/{n_iterations-1}")
            
            for agent in self.env.agents:
                # print(f"\nAgent {agent.id}:")
                start_pos = agent.pos
                # Find the closest node to the agent's position
                # TODO: this could be improved by dividing the enviroment in a grid and storing the closest node for each cell
                # Maybe this implementation will not be precise if the agent is very close to the border of the grid cell, but still
                
                #This if beause otherwise at each iteration the starting point would be recomputed, but the result would be the same always
                if agent.start_node is not None:
                    start_node = agent.start_node
                else:
                    start_node = min(range(n_points), key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(start_pos)))
                    agent.start_node = start_node
                
                path = [start_node]
                visited = set(path)
                
                # print("Starting while loop")
                while True:
                    current_node = path[-1]
                    # print("-----------------------------")
                    # print(f"> Current path: {path}")
                    # print(f"> Visited nodes: {visited}")
                    # print(f"> Current node: {current_node}")
                    if current_node in self.exit_nodes: # reached an exit node
                        # here we should check if the exit corresponds to the agent's target
                        # otherwise we should penalize the visit of this node !! (we do not want the agent to go out from the wrong exit)
                        # print(f"> Agent {agent.id} reached exit node {current_node} during simulation {iteration}.")
                        break
                    
                    neighbors = self.edges[current_node]
                    # print(f"> Neighbors of current node: {[n for n, _ in neighbors]}")
                    probabilities = []
                    for neighbor, cost in neighbors:
                        if neighbor not in visited:
                            tau = pheromone[current_node][neighbor] ** alpha
                            eta = 1 #(1.0 / cost) ** beta
                            probabilities.append(tau * eta)
                        else:
                            probabilities.append(0)
                            
                    # print(f"> Probabilities: {probabilities}")
                    
                    total = sum(probabilities)
                    if total == 0: # no unvisited neighbors
                        break
                    
                    probabilities = [p / total for p in probabilities]
                    # print(f"> Normalized probabilities: {probabilities}")
                    next_node = np.random.choice([n for n, _ in neighbors], p=probabilities)
                    # print(f"> Next node chosen: {next_node}")
                    
                    path.append(next_node)
                    visited.add(next_node)
                # print("Exited while loop")
                path_length = sum(np.linalg.norm(np.array(self.nodes[path[i]]) - np.array(self.nodes[path[i+1]])) for i in range(len(path)-1))
                # print(f"> Path found for agent {agent.id} during simulation {iteration}: {path} with length {path_length}")
                all_paths.append(path)
                all_path_lengths.append(path_length)
                
                if path[-1] in self.exit_nodes and (path_length < agent.path_length or agent.path is None):
                    agent.path = self.nodes_of(path)
                    agent.path_length = path_length
            
            # print("\nFinished agents in the loop. Now updating pheromone...")
            # Update pheromone
            pheromone *= (1 - evaporation_rate)
            # print("> Pheromone updating...")
            for path, length in zip(all_paths, all_path_lengths):
                if path[-1] in self.exit_nodes:
                    for i in range(len(path) - 1):
                        pheromone[path[i]][path[i+1]] += Q / length
                    
            # print(f"{pheromone}")
            # print("#########################################\n")
    
    def nodes_of(self, path_indices):
        return [np.array(self.nodes[i]) for i in path_indices]