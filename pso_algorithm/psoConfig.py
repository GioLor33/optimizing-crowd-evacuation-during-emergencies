from boids_algorithm.boidsConfig import BoidsConfig

class PSOConfig(BoidsConfig):
    def __init__(self, config_file):
        super().__init__(config_file)
        
        pso_section = self.config.get('algorithm-parameters', {}).get('pso-local', {})
        self.NEIGHBORHOOD_RADIUS = pso_section.get('neighborhood_radius', 5.0)
        self.W = pso_section.get('inertia_weight', 0.5)
        self.C1 = pso_section.get('cognitive_weight', 1.5)
        self.C2 = pso_section.get('social_weight', 1.5)