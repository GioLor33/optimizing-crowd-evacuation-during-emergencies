class Environment:
    def __init__(self, name, dimensions=(10,10), walls=[None], exits=[None]):
        self.name = name
        
        assert dimensions is not None, "Dimensions must be provided"
        assert dimensions[0] > 0 and dimensions[1] > 0, "Dimensions must be positive integers"
        self.__dimensions = dimensions
        
        # in self.grid we have:
        # -1 - safety exits
        # 0 - empty cell
        # 1 - wall/obstacle
        self.__grid = [[0 for _ in range(dimensions[1])] for _ in range(dimensions[0])]
        
        self.__exits = set()
        if exits != [None]:
            self.set_safety_exits(exits)
            
        self.__walls = set()
        if walls != [None]:
            self.set_walls(walls)


    def set_walls(self, positions):
        # If a wall is set on an exit, the wall is not entered into the environment
        if isinstance(positions, tuple):
            if not positions in self.__exits:
                self.__grid[positions[0]][positions[1]] = 1
                self.__walls.add(positions)
        if isinstance(positions, list):
            for pos in positions:
                if not pos in self.__exits:
                    self.__grid[pos[0]][pos[1]] = 1
                    self.__walls.add(pos)
        else:
            raise ValueError("Positions must be provided as a tuple or as a list of tuples")     

    def get_walls(self):
        return self.__walls
    
    def add_external_walls(self):
        rows, cols = self.__dimensions
        # top and bottom rows
        self.set_walls([(0, j) for j in range(cols)])
        self.set_walls([(rows - 1, j) for j in range(cols)])
        # left and right columns
        self.set_walls([(i, 0) for i in range(rows)])
        self.set_walls([(i, cols - 1) for i in range(rows)])
    
    def set_safety_exits(self, positions):
        # TODO: should we add a length=(x,y) parameter to specify exit size in x and y direction? In that case, we need to assert that len(length)==len(positions)
        if isinstance(positions, tuple):
            self.__grid[positions[0]][positions[1]] = -1
            self.__exits.add(positions)
        if isinstance(positions, list):
            for pos in positions:
                self.__grid[pos[0]][pos[1]] = -1
                self.__exits.add(pos)
        else:
            raise ValueError("Safety exits must be provided as a tuple or as a list of tuples")
             
    def get_safety_exits(self):
        return self.__exits

    def remove_safety_exit(self, position):
        if position in self.__exits:
            self.__grid[position[0]][position[1]] = 0
            self.__exits.remove(position)
    
    def get_dimensions(self):
        return self.__dimensions
    
    def get_grid(self):
        return self.__grid

    def __str__(self):
        return "\n".join(
            " ".join(f"{cell:2d}" for cell in row)
            for row in self.__grid
        )