from environments.agent import Agent
import numpy as np

class AcoAgent(Agent):
    def __init__(self, env_instance, uid):
        super().__init__(env_instance, uid)
        
        self.path = None
        self.path_length = 0.0
        self.start_node_id = None
        
        self.safe = False
        
        self.node_visited = set()
        
    def node_reached(self):
        self.path.pop(0)
        if len(self.path) == 0:
            self.safe = True
            return
        self.target = self.path[0]
