from environments.agent import Agent
import numpy as np
from environments.utils import segments_intersect, intersection_point, path_intersection_in_time

class AcoAgent(Agent):
    def __init__(self, env_instance, uid):
        super().__init__(env_instance, uid)
        self.max_speed = np.random.uniform(3.0, 5.0)
        init_v = (np.random.rand(2) - 0.5) * 2 # random initial velocity in range [-1,1)
        self.vel = self.max_speed * init_v / np.linalg.norm(init_v)
        self.vel_to_next_node = None
        
        self.prev_pos = self.pos.copy()
        self.future_pos = None
        self.path = None
        self.path_length = 0
        
        self.start_node_id = None
        
        self.safe = False
        
    def compute_velocity(self, dt):
        delta_pos = self.path[0] - self.pos
        self.vel = self.max_speed * delta_pos / np.linalg.norm(delta_pos)
        
        # check speed to reach next node
        self.vel_to_next_node = delta_pos / dt
        if self.max_speed > np.linalg.norm(self.vel_to_next_node):
            self.vel = self.vel_to_next_node
            
        self.future_pos = self.pos + self.vel * dt
        
    def move(self, dt):
        self.prev_pos = self.pos.copy()
        self.pos += self.vel * dt
        self.future_pos = None
        
        if np.allclose(self.vel, self.vel_to_next_node, atol=1e-3):
            self.node_reached()
                
    # def update(self, snapshot, dt):
        
    #     # step1: move in the direction of the next node in the path
    #     # step2: if reached the node, remove it from the path
    #     # step3: check collisions with walls and other agents (simple avoidance)
        
    #     self.prev_pos = self.pos.copy()
    #     delta_pos = self.path[0] - self.pos
    #     self.vel = self.max_speed * delta_pos / np.linalg.norm(delta_pos)
        
    #     vel_to_reach_next_node = delta_pos / dt
    #     if self.max_speed > np.linalg.norm(vel_to_reach_next_node):
    #         self.vel = vel_to_reach_next_node
    #     # self.avoid_walls(dt)
    #     self.avoid_agents(snapshot, dt)
    #     self.pos += self.vel * dt
        
    #     if np.allclose(self.vel, vel_to_reach_next_node, atol=1e-3):
    #         self.node_reached()
    
    # def avoid_agents(self, snapshot, dt):
    #     if self.future_pos is None:
    #         self.future_pos = self.pos + self.vel * dt
    #     for other_agent in snapshot:
    #         if other_agent.id == self.id:
    #             continue
    #         if other_agent.future_pos is None:
    #             other_agent.future_pos = other_agent.pos + other_agent.vel * dt
    #         if path_intersection_in_time(self.pos, self.vel, other_agent.pos, other_agent.vel, dt) is not None:
                
    #             n1 = np.linalg.norm(self.vel)
    #             if n1 == 0:
    #                 break
    #             n2 = np.linalg.norm(other_agent.vel)
    #             if n2 == 0:
    #                 break
    #             u1 = self.vel / np.linalg.norm(self.vel)
    #             u2 = other_agent.vel / np.linalg.norm(other_agent.vel)

    #             # Same direction → angle ≈ 0 → dot ≈ +1
    #             if np.dot(u1, u2) > 0.999:
    #                 if np.linalg.norm(self.vel) > np.linalg.norm(other_agent.vel):
    #                     self.vel = other_agent.vel
    #                 break
    #             else:
    #                 self.vel = self.vel * 0.5
    #             # if self.vel[0] == 0 and self.vel[1] == 0:
    #             #     break
    #             #print(f"{self.vel}. The agent is in position {self.pos}, other agent in position {other_agent.pos}. Exiting the while loop")
                
    #         # if segments_intersect(self.pos, self.future_pos, other_agent.pos, other_agent.future_pos):
    #         #     print(f"Agent {self.id} avoiding other agent {other_agent.id}. From velocity {self.vel} to ", end="")
    #         #     int_point = intersection_point(self.pos, self.future_pos, other_agent.pos, other_agent.future_pos)
    #         #     new_vel = (int_point - self.pos) * 0.9 / dt
    #         #     if np.sign(new_vel[0]) != np.sign(self.vel[0]):
    #         #         self.vel[0] = 0 
    #         #     else:
    #         #         self.vel[0] = new_vel[0]
    #         #     if np.sign(new_vel[1]) != np.sign(self.vel[1]):
    #         #         self.vel[1] = 0
    #         #     else:
    #         #         self.vel[1] = new_vel[1]
    #         #     #self.vel = (int_point - self.pos) * 0.9 / dt 
    #         #     print(f"{self.vel}. The agent is in position {self.pos}, other agent in position {other_agent.pos}, and the computed intersection point is {int_point}. The velocities were {self.vel} and {other_agent.vel}.")
    #         #     self.future_pos = self.pos + self.vel * dt 
                
    #     # TODO: se le velocità sono parallele, quello che devo fare è fare in modo che l'agente più veloce si sposti o a dx o a sx dell'altro, di poco, giusto per superarlo
            
    # def avoid_walls(self, dt):
    #     future_pos_self = self.pos + self.vel * dt
    #     walls = self.env.get_walls()
    #     for wall in walls:
    #         if segments_intersect(self.pos, future_pos_self, wall[0], wall[1]):
    #             self.vel = (intersection_point(self.pos, future_pos_self, wall[0], wall[1]) - self.pos) * 0.9 / dt 
    #             future_pos_self = self.pos + self.vel * dt
            
    def node_reached(self):
        self.path.pop(0)
        if len(self.path) == 0:
            self.safe = True