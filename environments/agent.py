import numpy as np

class Agent:
    def __init__(self, env_instance, uid):
        self.id = uid
        self.env = env_instance
        self.pos = np.array(self.env.get_random_spawn(), dtype=float)
        self.target = np.array(self.env.get_random_exit(), dtype=float)
        
    def get_position(self):
        return self.pos
    
    def update(self):
        pass