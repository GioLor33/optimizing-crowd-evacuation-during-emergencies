import yaml

class Config:
    def __init__(self, config_file):
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        self.file_path = config_file
            
        self.algorithm = self.config.get('algorithm', {}).get('name')
        self.dt = self.config.get('algorithm', {}).get('time-step')
        self.random_seed = self.config.get('algorithm', {}).get('seed', None)
        if self.random_seed is not None:
            self.random_seed = int(self.random_seed)
        print(f"RANDOM SEED SET TO {self.random_seed}")
        
        world = self.config.get('world', {})
        self.world_name = world.get('name')
        self.world_dimensions = world.get('dimensions')
        self.exits = [
            [tuple(float(coord) for coord in exit[0]), tuple(float(coord) for coord in exit[1])]
            for exit in world.get('exits', [])
        ]
        if world.get('walls') is None:
            self.walls = []
        else:
            self.walls = [
                [tuple(float(coord) for coord in wall[0]), tuple(float(coord) for coord in wall[1])]
                for wall in world.get('walls', [])
            ] 
        self.num_agents = int(world.get('num-agents'))
        
        visualization = self.config.get('visualization', {})
        self.visualization = visualization.get('activate')
        if self.visualization:
            self.window_name = visualization.get('window-name', 'Crowd Simulation')
            self.visualization_dimensions = tuple(
                int(dim) for dim in visualization.get('dimensions')
            )            
            
            self.padding = int(visualization.get('padding'))
            self.right_border = int(visualization.get('right-border'))
            self.left_border = int(visualization.get('left-border'))
            self.top_border = int(visualization.get('top-border'))
            self.bottom_border = int(visualization.get('bottom-border'))
            
            self.background_color = visualization.get('background-color', [0, 0, 0])
            self.agent_color = self.parse_color(visualization.get('agent-color', [0, 102, 255]), default=[0, 102, 255])
            self.wall_color = self.parse_color(visualization.get('wall-color', [200, 200, 200]), default=[200, 200, 200])
            self.exit_color = self.parse_color(visualization.get('exit-color', [57, 255, 20]), default=[57, 255, 20])
            self.title_color = self.parse_color(visualization.get('title-color', [255, 0, 0]), default=[255, 0, 0])
            self.text_color = self.parse_color(visualization.get('text-color', [255, 255, 255]), default=[255, 255, 255])
    
    def get(self, key, default=None):
        return self.config.get(key, default)
    
    def parse_color(self, value, default=None):
        # TODO: consider raylib asks for colors in RGBA format. For now opacity is set to 255
        
        if isinstance(value, str):
            try:
                return [int(x) for x in value.replace('(', '').replace(')', '').split(',')]
            except Exception:
                print("Error parsing color from string in config file. Using default value.")
                return default
        elif isinstance(value, (list, tuple)):
            return [int(x) for x in value] + [255]
        else:
            print("Error parsing color in config file. Using default value.")
            return default