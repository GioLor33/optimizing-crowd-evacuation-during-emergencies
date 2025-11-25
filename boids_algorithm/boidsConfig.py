from parser.config import Config

class BoidsConfig(Config):
    def __init__(self, config_file):
        super().__init__(config_file)
        
        agent_params = self.config.get('algorithm-parameters', {}).get('boids', {}).get('agents', {})
        self.AGENT_SIZE = float(agent_params.get('size'))
        self.VISION_RADIUS = float(agent_params.get('vision-radius'))
        self.MIN_SEPARATION = float(agent_params.get('min-separation'))
        self.WALL_AVOID_DISTANCE = float(agent_params.get('wall-avoid-distance'))
        self.SPEED_LIMIT = float(agent_params.get('max-speed'))
        self.FORCE_LIMIT = float(agent_params.get('max-force'))

        weights = self.config.get('algorithm-parameters', {}).get('boids', {}).get('weights', {})
        self.W_SEEK = float(weights.get('seek'))
        self.W_AVOID = float(weights.get('avoid'))
        self.W_SEPARATE = float(weights.get('separation'))
        self.W_ALIGN = float(weights.get('alignment'))
        self.W_COHERE = float(weights.get('cohesion'))