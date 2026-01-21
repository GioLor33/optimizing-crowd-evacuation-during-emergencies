import yaml

class Config:
    
    def __init__(self, config_file = None):
        
        if config_file is None:
            self.config = dict()
            return
        
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        self.file_path = config_file
            
        self.algorithm = self.config.get('algorithm', {}).get('name')
        if self.algorithm == 'boids':
            self.parse_boids_algorithm_params()
        elif self.algorithm == 'aco':
            self.parse_aco_algorithm_params()
        elif self.algorithm == 'pso':
            self.parse_pso_algorithm_params()
        else:
            raise ValueError("Algorithm " + str(self.algorithm) + " not recognized.")
            exit(1)
        
        self.dt = self.config.get('algorithm', {}).get('time-step')
        self.random_seed = self.config.get('algorithm', {}).get('seed', None)
        if self.random_seed is not None:
            self.random_seed = int(self.random_seed)
        
        world = self.config.get('world', {})
        self.world_name = world.get('name')
        self.world_type = world.get('type', 'custom')
        self.num_agents = int(world.get('num-agents'))
        # self.spawn_agent_method = world.get('spawn')
        if self.world_type == 'custom':
            self.parse_custom_world(world)
            
        # Visualizers
        visualization = self.config.get('visualization', {})
        self.visualization = visualization.get('activate')
        if self.visualization:
            self.parse_visualization_params(visualization)
            
    
    def get(self, key, default=None):
        return self.config.get(key, default)
    
    def parse_color(self, value, default=None):
        
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
        
    def parse_boids_algorithm_params(self):
        agent_params = self.config.get('algorithm-parameters', {}).get('boids', {}).get('agents', {})
        self.vision_radius = float(agent_params.get('vision-radius'))
        self.min_separation = float(agent_params.get('min-separation'))
        self.speed_limit = float(agent_params.get('max-speed'))
        self.force_limit = float(agent_params.get('max-force'))

        weights = self.config.get('algorithm-parameters', {}).get('boids', {}).get('weights', {})
        self.weights = {
            'seek': float(weights.get('seek')),
            'avoid': float(weights.get('avoid')),
            'separate': float(weights.get('separation')),
            'align': float(weights.get('alignment')),
            'cohere': float(weights.get('cohesion'))
        }
    
    def parse_aco_algorithm_params(self):
        aco = self.config.get('algorithm-parameters', {}).get('aco', {})
        
        self.num_ants = int(aco.get('num-ants-in-simulation'))
        self.num_iterations = int(aco.get('num-iterations'))
        self.alpha = float(aco.get('alpha'))
        self.beta = float(aco.get('beta'))
        self.evaporation_rate = float(aco.get('evaporation-rate'))
    
        self.graph_type = aco.get('graph-type')
        self.n = aco.get('n')
        self.m = aco.get('m')
        self.k_connectivity = aco.get('k-connectivity')
        
    def parse_pso_algorithm_params(self):
        pso_section = self.config.get('algorithm-parameters', {}).get('pso', {})
        self.neighborhood_radius = pso_section.get('neighborhood_radius', 5.0)
        self.W = pso_section.get('inertia_weight', 0.5)
        self.C1 = pso_section.get('cognitive_weight', 1.5)
        self.C2 = pso_section.get('social_weight', 1.5)
    
    def parse_custom_world(self, world):
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
            
    def parse_visualization_params(self, visualization):
        self.window_name = visualization.get('window-name', 'Crowd Simulation')        
        
        self.background_color = visualization.get('background-color', [0, 0, 0])
        self.wall_color = self.parse_color(visualization.get('wall-color', [200, 200, 200]), default=[200, 200, 200])
        self.exit_color = self.parse_color(visualization.get('exit-color', [57, 255, 20]), default=[57, 255, 20])
        self.title_color = self.parse_color(visualization.get('title-color', [255, 0, 0]), default=[255, 0, 0])
        self.text_color = self.parse_color(visualization.get('text-color', [255, 255, 255]), default=[255, 255, 255])