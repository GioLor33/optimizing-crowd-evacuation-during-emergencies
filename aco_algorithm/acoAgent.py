from environments.agent import Agent
import numpy as np
from environments.utils import segments_intersect, intersection_point

class AcoAgent(Agent):
    def __init__(self, env_instance, uid):
        super().__init__(env_instance, uid)
        self.max_speed = np.random.uniform(4.0, 6.0)
        init_v = (np.random.rand(2) - 0.5) * 2 # random initial velocity in range [-1,1)
        self.vel = self.max_speed * init_v / np.linalg.norm(init_v)
        
        self.prev_pos = self.pos.copy()
        self.path = None
        self.path_length = 0
        
        self.safe = False
                
    def update(self, snapshot, dt):
        
        # step1: move in the direction of the next node in the path
        # step2: if reached the node, remove it from the path
        # step3: check collisions with walls and other agents (simple avoidance)
        
        # collision avoidance si fa dimuendo la velocità (facciamo vel / 2) se c'è un agente con cui mi schianterei nei passo successivo
        # questo perchè sto effettivamente simulando quello che avverrebbe nella realtà
        
        self.prev_pos = self.pos.copy()
        delta_pos = self.path[0] - self.pos
        self.vel = self.max_speed * delta_pos / np.linalg.norm(delta_pos)
        
        vel_to_reach_next_node = delta_pos / dt
        if self.max_speed > np.linalg.norm(vel_to_reach_next_node):
            self.vel = vel_to_reach_next_node
        # self.avoid_walls(dt)
        self.avoid_agents(snapshot, dt)
        self.pos += self.vel * dt
        
        if np.allclose(self.vel, vel_to_reach_next_node, atol=1e-3):
            self.node_reached()
    
    def avoid_agents(self, snapshot, dt):
        future_pos_self = self.pos + self.vel * dt
        for other_agent in snapshot:
            if other_agent.id == self.id:
                continue
            future_pos_other = other_agent.pos + other_agent.vel * dt
            if segments_intersect(self.pos, future_pos_self, other_agent.pos, future_pos_other):
                self.vel = (intersection_point(self.pos, future_pos_self, other_agent.pos, future_pos_other) - self.pos) * 0.9 / dt 
                future_pos_self = self.pos + self.vel * dt 
            
    def avoid_walls(self, dt):
        future_pos_self = self.pos + self.vel * dt
        walls = self.env.get_walls()
        for wall in walls:
            if segments_intersect(self.pos, future_pos_self, wall[0], wall[1]):
                self.vel = (intersection_point(self.pos, future_pos_self, wall[0], wall[1]) - self.pos) * 0.9 / dt 
                future_pos_self = self.pos + self.vel * dt
            
    def node_reached(self):
        self.path.pop(0)
        self.path_length = len(self.path)
        if self.path_length == 0:
            self.safe = True