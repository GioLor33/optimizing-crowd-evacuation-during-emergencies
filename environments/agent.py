import numpy as np

class Agent:
    def __init__(self, env_instance, uid):
        self.id = uid
        self.env = env_instance
        self.pos = np.array(self.env.get_random_spawn(), dtype=float)
        
        # # TO CHANGE
        # self.pos = [uid, env_instance.get_height() // 2]
        # # END TO CHANGE
        
        self.target = np.array(self.env.get_random_exit(), dtype=float)
        
    def get_position(self):
        return self.pos
    
    def update(self):
        pass
    
    # Agent 7 avoiding other agent 4. 
    # From velocity [4.7039595 0.       ] to [-71.0644128    8.34346024]. 
    # The agent is in position [5.54095534 5.        ], other agent in position [5.1303923  5.04169061], 
    # and the computed intersection point is [5.14615305 5.04635256]. 
    # The velocities were [-71.0644128    8.34346024] and [-3.00989529 -0.96236019].

