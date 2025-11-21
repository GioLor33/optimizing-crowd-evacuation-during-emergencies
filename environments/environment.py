class Environment:
    def __init__(self, name, dimensions=(10,10), walls=[None], exits=[None]):
        self.name = name
        
        assert dimensions is not None, "Dimensions must be provided"
        assert dimensions[0] > 0 and dimensions[1] > 0, "Dimensions must be positive integers"
        self.__dimensions = dimensions
        
        self.__exits = set()
        if exits != [None]:
            self.set_safety_exits(exits)
            
        self.__walls = set()
        if walls != [None]:
            self.set_walls(walls)


    def set_walls(self, list_of_walls):
        if isinstance(list_of_walls, list):
            for wall in list_of_walls:
                assert isinstance(wall, list) and len(wall) == 2, "Wall positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                for point in wall:
                    assert isinstance(point, tuple) and len(point) == 2, "Wall positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                self.__walls.add((tuple(wall[0]), tuple(wall[1])))
        else:
            raise ValueError("Positions must be provided as a tuple or as a list of tuples")  
           
        # TODO: how do we manage if a wall is added on top of an exit? Should it overwrite the exit?
        # I think a check is actually needed when adding walls or exits to avoid conflicts.

    def get_walls(self):
        return self.__walls
    
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
                assert isinstance(exit, list) and len(exit) == 2, "Safety exit positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                for point in exit:
                    assert isinstance(point, tuple) and len(point) == 2, "Safety exit positions must be provided as a list of two tuples indicating the starting and ending point of the wall"
                self.__exits.add((tuple(exit[0]), tuple(exit[1])))
        else:
            raise ValueError("Safety exit positions must be provided as a tuple or as a list of tuples")  
             
    def get_safety_exits(self):
        return self.__exits

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