from environments.utils import segments_intersect
import numpy as np

class Environment:
    def __init__(self, name, dimensions=(10,10), walls=[None], exits=[None]):
        self.name = name
        
        assert dimensions is not None, "Dimensions must be provided"
        assert dimensions[0] > 0 and dimensions[1] > 0, "Dimensions must be positive integers"
        self.__dimensions = dimensions       

        self.__walls = list()
        if walls != [None]:
            self.set_walls(walls)
        self.add_external_walls()
        
        self.__exits = list()
        if exits != [None]:
            self.set_safety_exits(exits)
            
        self.agents = []
        self.initial_agent_count = 0
        self.simulation_time = 0.0
        
    def add_agent(self, agent):
        self.agents.append(agent)
        self.initial_agent_count += 1
        
    def set_agents(self, agents):
        self.agents = agents
        self.initial_agent_count = len(agents)
    
    def get_agents(self):
        return self.agents

    def set_walls(self, list_of_walls):
        if isinstance(list_of_walls, list):
            for wall in list_of_walls:
                assert isinstance(wall, list) and len(wall) == 2, "Wall positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                for point in wall:
                    assert isinstance(point, tuple) and len(point) == 2, "Wall positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                self.__walls.append((tuple(wall[0]), tuple(wall[1])))
        else:
            raise ValueError("Positions must be provided as a tuple or as a list of tuples")  
           
    def get_walls(self):
        return self.__walls
    
    def get_wall(self, i:int):
        return self.__walls[i]
    
    def add_external_walls(self):
        width, height = self.__dimensions
        # top and bottom rows
        self.set_walls([[(0, 0), (0, height)]])
        self.set_walls([[(0, 0), (width, 0)]])
        self.set_walls([[(0, height), (width, height)]])
        self.set_walls([[(width, 0), (width, height)]])
    
    def set_safety_exits(self, list_of_exits):
        if isinstance(list_of_exits, list):
            for exit in list_of_exits:
                assert isinstance(exit, list) and len(exit) == 2, "Safety exit positions must be provided as a list of two tuples"
                for point in exit:
                    assert isinstance(point, tuple) and len(point) == 2, "Safety exit positions must be provided as a list of two tuples"
                
                exit_tuple = (tuple(exit[0]), tuple(exit[1]))

                # walls_to_add = set()
                # walls_to_remove = set()
                # walls = self.__walls
                
                N = len(self.__walls)
                for i in range(N-1, -1, -1):
                    wall = self.__walls[i]
                    wall_start, wall_end = wall
                    exit_start, exit_end = exit_tuple
                    
                    # Verify if wall and exit overlap
                    if wall_start[0] == wall_end[0] == exit_start[0] == exit_end[0]:  # vertical
                        if not (exit_end[1] <= wall_start[1] or exit_start[1] >= wall_end[1]):
                            self.__walls.remove(wall)
                            # walls_to_remove.add(wall)
                            if wall_start[1] < exit_start[1]:
                                self.__walls.append((wall_start, (wall_start[0], exit_start[1])))
                                #walls_to_add.add((wall_start, (wall_start[0], exit_start[1])))
                            if wall_end[1] > exit_end[1]:
                                self.__walls.append(((wall_end[0], exit_end[1]), wall_end))
                                #walls_to_add.add(((wall_end[0], exit_end[1]), wall_end))
                    elif wall_start[1] == wall_end[1] == exit_start[1] == exit_end[1]:  # horizontal
                        if not (exit_end[0] <= wall_start[0] or exit_start[0] >= wall_end[0]):
                            #walls_to_remove.add(wall)
                            self.__walls.remove(wall)
                            if wall_start[0] < exit_start[0]:
                                self.__walls.append((wall_start, (exit_start[0], wall_start[1])))
                                #walls_to_add.add((wall_start, (exit_start[0], wall_start[1])))
                            if wall_end[0] > exit_end[0]:
                                self.__walls.append(((exit_end[0], wall_end[1]), wall_end))
                                #walls_to_add.add(((exit_end[0], wall_end[1]), wall_end))

                # Update walls
                # self.__walls.difference_update(walls_to_remove)
                # self.__walls.update(walls_to_add)
                
                #self.__exits.add(exit_tuple)
                self.__exits.append(exit_tuple)
        else:
            raise ValueError("Safety exit positions must be provided as a tuple or as a list of tuples")
  
             
    def get_safety_exits(self):
        return self.__exits
    
    def get_safety_exits_number(self):
        return len(self.__exits)

    def get_exit_centroids(self):
        """Returns a list of (x,y) coordinates representing the centers of the exits"""
        centroids = []
        for exit_line in self.__exits:
            p1, p2 = exit_line
            midpoint = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
            centroids.append(midpoint)
        return centroids

    def remove_safety_exit(self, position):
        if position in self.__exits:
            self.__exits.remove(position)
    
    def get_dimensions(self):
        return self.__dimensions
    
    def get_width(self):
        return self.__dimensions[0]
    
    def get_height(self):
        return self.__dimensions[1]

    def __str__(self):
        return "Environment '{}': \n > dimensions={}, \n > # of walls={}, \n > # of exits={}".format(
                self.name,
                self.__dimensions,
                len(self.__walls),
                len(self.__exits),
            )
        
    def check_something_reached(self, prev_pos, pos, name):
        if name == "exit":
            to_check = self.__exits
        elif name == "wall":
            to_check = self.__walls
        else:
            raise ValueError("Unknown name provided to check_something_reached: {}".format(name))
        
        if pos is None or prev_pos is None:
            raise ValueError("Positions provided to check_something_reached cannot be None")
        
        # Agent out of the environment bounds
        # if pos[0] < 0 or pos[0] > self.__dimensions[0] or pos[1] < 0 or pos[1] > self.__dimensions[1]:
        #     return None
        
        for i, item in enumerate(to_check):
            if segments_intersect(prev_pos, pos,
                                  item[0], item[1]):
                return i
        return None
    
    def check_is_position_free(self, position):
        for wall in self.__walls:
            if segments_intersect(position, position,
                                  wall[0], wall[1]):
                return False
        for exit in self.__exits:
            if segments_intersect(position, position,
                                  exit[0], exit[1]):
                return False
        return True
    
    
    def closest_point_on_segment(self, P, A, B):
    
        A = np.array(A, float)
        B = np.array(B, float)
        P = np.array(P, float)
        
        AB = B - A
        if np.allclose(AB, 0):
            return A
        t = np.dot(P - A, AB) / np.dot(AB, AB)
        t = np.clip(t, 0, 1)
        return A + t * AB
    
    def has_reached_exit(self, agent, margin=0.5):
        for exit_seg in self.get_safety_exits():
            closest = self.closest_point_on_segment(agent.pos, exit_seg[0], exit_seg[1])
            if np.linalg.norm(agent.pos - closest) <= margin:
                return exit_seg
        return None
    
    
    
    ###########################################
    
    def get_random_spawn(self):
        gx = np.random.uniform(1, self.__dimensions[0] - 2)
        gy = np.random.uniform(1, self.__dimensions[1] - 2)
        if self.check_is_position_free((gx, gy)):
            return (gx, gy)

    def get_random_exit(self, overshoot=0.5):
        if not self.__exits:
            raise ValueError("No exits defined in the environment")

        exits = list(self.__exits)
        idx = np.random.choice(len(exits))
        A, B = exits[idx]
        point = (
            (A[0] + B[0]) / 2,
            (A[1] + B[1]) / 2
        )
        return point
        