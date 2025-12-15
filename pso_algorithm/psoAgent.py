import numpy as np
from environments.agent import Agent
from collections import deque
from environments.utils import segments_intersect

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

    def update(self, agents_snapshot, env, dt, A=2.0, B=0.5, k=1.2e5, kappa=2.4e5, tau=0.5):

        # PSO
        lbest_position = self._compute_lbest(agents_snapshot)
        r1, r2 = np.random.rand(), np.random.rand()
        pso_velocity = self.w * self.vel \
                    + self.c1 * r1 * (self.pbest_position - self.pos) \
                    + self.c2 * r2 * (lbest_position - self.pos)

        # Pedestrian dynamics
        f_agents = self.repulsive_force(
            agents_snapshot,
            A=A, B=B, k=k, kappa=kappa
        )

        f_walls = self.obstacle_force(
            env.get_walls(),
            A=A, B=B, k=k, kappa=kappa
        )

        self.f_desired = pso_velocity
        # With driving force only if the exit is visible
        if self.target is None:
            for exit in env.get_safety_exits():
                target_center = ( (exit[0][0] + exit[1][0]) / 2, (exit[0][1] + exit[1][1]) / 2 )
                if self.is_visible(target_center, env.get_walls()):
                    self.target = self.closest_point_on_segment(self.pos, exit[0], exit[1])
                    self.f_desired = self.driving_force() 
                    break
        else:  
            self.f_desired = self.driving_force()
       
        self.f_walls = f_walls
        self.f_agents = f_agents
        
        self.vel += dt * (self.f_desired + (self.f_agents + self.f_walls) / self.mass)

        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
            self.vel = (self.vel / speed) * self.max_speed

        self.pos += self.vel * dt

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
                if self.is_visible(other.pos, self.env.get_walls()):
                    neighbors.append(other)

        if not neighbors:
            return self.pos.copy() 

        best_neighbor = min(neighbors, key=lambda a: getattr(a, 'pbest_time', float('inf')))
        return best_neighbor.pbest_position.copy()
    
    def is_visible(self, target_pos, walls):
        for wall_start, wall_end in walls:
            if segments_intersect(self.pos, target_pos, wall_start, wall_end):
                return False
        return True
    

# Fitness implementation using a grid-based approach
class GridFitness:
    def __init__(self, environment): 
        self.env = environment
        self.grid_height = self.env.get_dimensions()[1]
        self.grid_width = self.env.get_dimensions()[0]
                
        self.width, self.height = self.env.get_dimensions()
        self.cell_width = self.width / self.grid_width
        self.cell_height = self.height / self.grid_height
        
        # 0 = free, 1 = wall
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=int)
        self._draw_walls()
        self._ensure_exit_free_cells()
        
        self.distance_map = np.full((self.grid_width, self.grid_height), np.inf)
        self._compute_distance_map()
        # self.plot_fitness()
          

    # Bresenham's line algorithm to draw walls on the grid
    def _draw_line(self, i1, j1, i2, j2):
        """Draw a thin wall using Bresenham line algorithm on the grid."""
        dx = abs(i2 - i1)
        dy = abs(j2 - j1)
        x, y = i1, j1
        sx = 1 if i1 < i2 else -1
        sy = 1 if j1 < j2 else -1

        if dx > dy:
            err = dx / 2.0
            while x != i2:
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    self.grid[x, y] = 1
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != j2:
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    self.grid[x, y] = 1
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        # final point
        if 0 <= i2 < self.grid_width and 0 <= j2 < self.grid_height:
            self.grid[i2, j2] = 1


    def _draw_walls(self):
        for wall in self.env.get_walls():
            (x1, y1), (x2, y2) = wall
            i1, j1 = self.world_to_grid((x1, y1))
            i2, j2 = self.world_to_grid((x2, y2))
            self._draw_line(i1, j1, i2, j2)
    
    def world_to_grid(self, pos):
        x, y = pos
        i = int(x / self.cell_width)
        j = int(y / self.cell_height)
        # Clamp to grid bounds
        i = max(0, min(self.grid_width - 1, i))
        j = max(0, min(self.grid_height - 1, j))
        return (i,j)
    
    def _compute_distance_map(self):
        queue = deque()
        visited = np.zeros((self.grid_width, self.grid_height), dtype=bool)
        
        for exit_seg in self.env.get_safety_exits():
            (x1, y1), (x2, y2) = exit_seg
            exit_x = (x1 + x2) / 2
            exit_y = (y1 + y2) / 2
            ei, ej = self.world_to_grid((exit_x, exit_y))
            
            queue.append((ei, ej, 0))
            visited[ei, ej] = True
            self.distance_map[ei, ej] = 0

        # Choose the number of directions (4 or 8)
        directions = [(-1,0),(1,0),(0,-1),(0,1)]
        # directions = [
        #     (-1,0),(1,0),(0,-1),(0,1),
        #     (-1,-1),(-1,1),(1,-1),(1,1)
        # ]
        
        # BFS
        while queue:
            i, j, dist = queue.popleft()
            for di, dj in directions: 
                ni, nj = i + di, j + dj
                if 0 <= ni < self.grid_width and 0 <= nj < self.grid_height:
                    if not visited[ni, nj] and self.grid[ni, nj] == 0:
                        visited[ni, nj] = True
                        self.distance_map[ni, nj] = dist + 1
                        queue.append((ni, nj, dist + 1))
    
    def compute_fitness(self, pos):
        i, j = self.world_to_grid(pos)
        dist = self.distance_map[i, j]
        return float(dist) if not np.isinf(dist) else float("inf")

        
    def _ensure_exit_free_cells(self):
        for exit_seg in self.env.get_safety_exits():
            (x1, y1), (x2, y2) = exit_seg
            exit_x = (x1 + x2)/2
            exit_y = (y1 + y2)/2
            ei, ej = self.world_to_grid((exit_x, exit_y))
            self.grid[ei, ej] = 0  

    def plot_fitness(self):
        import matplotlib.pyplot as plt

        data = self.distance_map.copy()

        # Walls
        data[self.grid == 1] = -1

        plt.figure(figsize=(8, 8))
        cmap = plt.cm.viridis
        cmap.set_under('black')   

        plt.imshow(data.T, origin='upper', cmap=cmap, vmin=0)

        plt.colorbar(label="Distance to Nearest Exit")
        plt.title("Fitness Distance Map")
        plt.show()


 