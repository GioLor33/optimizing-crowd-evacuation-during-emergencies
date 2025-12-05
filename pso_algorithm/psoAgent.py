import numpy as np
#from boids_algorithm.boidsAgent import BoidsAgent
from environments.agent import Agent
from collections import deque

class LocalPSOAgent(Agent):
    def __init__(self, env_instance, uid, config, fitness_map):
        super().__init__(env_instance, uid)
        self.pbest_position = self.pos.copy()
        self.pbest_time = float('inf')  
        self.neighborhood_radius = config.NEIGHBORHOOD_RADIUS
        self.w = config.W
        self.c1 = config.C1
        self.c2 = config.C2
        self.fitness_map = fitness_map
        self.prev_pos = None

    def update(self, agents_snapshot, env, dt):
        super().update(agents_snapshot, env, dt)
        acc = np.zeros(2)
        self.prev_pos = self.pos.copy()

        # PSO 
        lbest_position = self._compute_lbest(agents_snapshot)
        r1, r2 = np.random.rand(), np.random.rand()

        pso_velocity = self.w * self.vel + self.c1 * r1 * (self.pbest_position - self.pos) + self.c2 * r2 * (lbest_position - self.pos)
        acc += pso_velocity

        self.vel += acc * dt
        self.vel = self._limit_vector(self.vel, self.max_speed)
        self.pos += self.vel * dt

        # t_exit_estimate = self._estimate_time_to_exit()
        # if t_exit_estimate < self.pbest_time:
        #     self.pbest_time = t_exit_estimate
        #     self.pbest_position = self.pos.copy()
                
        fitness = self.fitness_map.compute_fitness(self.pos)
        if fitness < self.pbest_time:   
            self.pbest_time = fitness
            self.pbest_position = self.pos.copy()

    def _limit_vector(self, vector, max_val):
        norm = np.linalg.norm(vector)
        if norm > max_val and norm > 0:
            return (vector / norm) * max_val
        return vector


    def _compute_lbest(self, agents_snapshot):
        """
        Compute the local best position among neighboring agents.
        """
        neighbors = []
        for other in agents_snapshot:
            if other.id == self.id:
                continue
            dist = np.linalg.norm(self.pos - other.pos)
            if dist <= self.neighborhood_radius:
                neighbors.append(other)

        if not neighbors:
            return self.pos.copy() 

        best_neighbor = min(neighbors, key=lambda a: getattr(a, 'pbest_time', float('inf')))
        return best_neighbor.pbest_position.copy()

    # def _estimate_time_to_exit(self):
    #     """
    #     Simple heuristic: estimate time to exit based on distance to nearest exit.
    #     """
    #     exits = [((ex[0][0]+ex[1][0])/2, (ex[0][1]+ex[1][1])/2) 
    #              for ex in self.env.get_safety_exits()]
    #     if not exits:
    #         return float('inf')
    #     distances = [np.linalg.norm(np.array(self.pos) - np.array(ex)) for ex in exits]
    #     return min(distances)

# Fitness implementation using a grid-based approach
class GridFitness:
    def __init__(self, environment, grid_size=100): # TO DO: make grid size configurable
        self.env = environment
        self.grid_size = grid_size
        
        self.width, self.height = self.env.get_dimensions()
        self.cell_width = self.width / grid_size
        self.cell_height = self.height / grid_size
        
        # 0 = free, 1 = wall
        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self._draw_walls()
        self._ensure_exit_free_cells()
        
        self.distance_map = np.full((grid_size, grid_size), np.inf)
        self._compute_distance_map()
    

    def _draw_walls(self):
        for wall in self.env.get_walls():
            (x1, y1), (x2, y2) = wall
            i1, j1 = int(x1 / self.cell_width), int(y1 / self.cell_height)
            i2, j2 = int(x2 / self.cell_width), int(y2 / self.cell_height)
            for i in range(min(i1,i2), max(i1,i2)+1):
                for j in range(min(j1,j2), max(j1,j2)+1):
                    if 0 <= i < self.grid_size and 0 <= j < self.grid_size:
                        self.grid[i,j] = 1 
    
    def world_to_grid(self, pos):
        x, y = pos
        i = int(x / self.cell_width)
        j = int(y / self.cell_height)
        # Clamp to grid bounds
        i = max(0, min(self.grid_size - 1, i))
        j = max(0, min(self.grid_size - 1, j))
        return (i,j)
    
    def _compute_distance_map(self):
        queue = deque()
        visited = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        
        for exit_seg in self.env.get_safety_exits():
            (x1, y1), (x2, y2) = exit_seg
            exit_x = (x1 + x2) / 2
            exit_y = (y1 + y2) / 2
            ei, ej = self.world_to_grid((exit_x, exit_y))
            if 0 <= ei < self.grid_size and 0 <= ej < self.grid_size:
                queue.append((ei, ej, 0))  # i, j, distance
                visited[ei, ej] = True
                self.distance_map[ei, ej] = 0
        
        # BFS
        while queue:
            i, j, dist = queue.popleft()
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:  # 4-directions
                ni, nj = i + di, j + dj
                if 0 <= ni < self.grid_size and 0 <= nj < self.grid_size:
                    if not visited[ni, nj] and self.grid[ni, nj] == 0:
                        visited[ni, nj] = True
                        self.distance_map[ni, nj] = dist + 1
                        queue.append((ni, nj, dist + 1))
    
    def compute_fitness(self, pos):
        i, j = self.world_to_grid(pos)
        if 0 <= i < self.grid_size and 0 <= j < self.grid_size:
            dist = self.distance_map[i,j]
            if np.isinf(dist):
                return float('inf')  # unreachable
            return dist
        else:
            return float('inf')  # out of bound
        
    def _ensure_exit_free_cells(self):
        for exit_seg in self.env.get_safety_exits():
            (x1, y1), (x2, y2) = exit_seg
            exit_x = (x1 + x2)/2
            exit_y = (y1 + y2)/2
            ei, ej = self.world_to_grid((exit_x, exit_y))
            self.grid[ei, ej] = 0  