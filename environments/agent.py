import numpy as np

class Agent:
    def __init__(self, env_instance, uid):
        self.id = uid
        self.env = env_instance
        
        self.radius = np.random.uniform(0.2, 0.4)
        
        self.pos = np.array(self.env.get_random_spawn(agent=self), dtype=float)
        self.global_target = np.array(self.env.get_random_exit(), dtype=float)
        self.target = None

        self.max_speed = np.random.uniform(3.0, 5.0)
        init_v = (np.random.rand(2) - 0.5) * 2 # random initial velocity in range [-1,1)
        self.vel = self.max_speed * init_v / np.linalg.norm(init_v)
        self.mass = np.random.uniform(45.0, 75.0)
        self.tau = 0.5  
        
        self.f_desired = np.zeros(2)
        self.f_walls = np.zeros(2)
        self.f_agents = np.zeros(2)
        
        self.color = (
            np.random.randint(50, 255),
            np.random.randint(50, 255),
            np.random.randint(50, 255)
        )
        
        self.safe = False
        self.fail = False
        
    def get_position(self):
        return self.pos
    
    def update(self, agent_snapshot, env, dt=0.05, A=2.0, B=0.08, k=1.2e5, kappa=2.4e5, tau=0.5): # TODO: global variables?
       
        if self.target is None:
            raise ValueError("Agent " + str(self.id) + " has no target assigned. If no specific target is needed, set target to global target.")
       
        # x, y = self.target
        # closest_point = self.closest_point_on_segment(self.pos, x, y)
        # direction = closest_point - self.pos
        # dist_target = np.linalg.norm(direction)

        # if dist_target > 1e-8:
        #     direction /= dist_target # normalizing the direction to obtain a unit vector
        # else:
        #     direction = np.zeros(2)  # already at the target

        # v_desired = direction * self.max_speed
        f_desired = self.driving_force()
        
        f_agents = self.repulsive_force(
            agent_snapshot,
            A=A, B=B, k=k, kappa=kappa
        )

        f_walls = self.obstacle_force(
            env.get_walls(),
            A=A, B=B, k=k, kappa=kappa
        )

        self.f_desired = f_desired
        self.f_walls = f_walls
        self.f_agents = f_agents
        
        #total_force = f_desired + f_agents + f_walls

        self.vel += dt * (f_desired + (f_agents + f_walls) / self.mass)

        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
            self.vel = (self.vel / speed) * self.max_speed

        self.pos = self.pos + self.vel * dt


    # Function described in https://pedestriandynamics.org/models/social_force_model/
    def driving_force(self):
        x, y = self.target
        if len(self.target) == 2:
            closest_point = self.target
        else:
            closest_point = self.closest_point_on_segment(self.pos, x, y)

        direction = closest_point - self.pos
        norm = np.linalg.norm(direction)

        if norm < 1e-8:
            #direction = self.vel / (np.linalg.norm(self.vel) + 1e-8)  # avoid division by zero
            direction = np.zeros(2) 
        else:
            direction /= norm 
            
        v_des = direction * self.max_speed
        return (v_des - self.vel) / self.tau
    

    def repulsive_force(self, agents, A=2.0, B=0.5, k=1.2e5, kappa=2.4e5):
        total = np.zeros(2)
        for other in agents:
            if other is self:
                continue
            r_j = other.radius if hasattr(other, "radius") else self.radius
            total += self._repulsion_from_point(
                p_j = other.pos,
                v_j = other.vel,
                r_j = r_j,
                A=A, B=B, k=k, kappa=kappa
            )
        return total


    def obstacle_force(self, walls, A=2., B=0.5, k=1.2e5, kappa=2.4e5):
        total = np.zeros(2)

        for (wA, wB) in walls:
            wA = np.array(wA, float)
            wB = np.array(wB, float)
            seg = wB - wA
            seg_len_sq = np.dot(seg, seg)

            if seg_len_sq == 0:
                closest = wA
            else:
                t = np.dot(self.pos - wA, seg) / seg_len_sq
                t = np.clip(t, 0, 1)
                closest = wA + t * seg

            total += self._repulsion_from_point(
                p_j = closest,
                v_j = np.zeros(2),
                r_j = 0.0,
                A=A, B=B, k=k, kappa=kappa,
            )
        
        return total


    def _repulsion_from_point(self, p_j, v_j, r_j, A=2.0, B=0.5, k=1.2e5, kappa=2.4e5):
        d_vec = self.pos - p_j
        dist = np.linalg.norm(d_vec)

        # Collision handling
        if dist < 1e-8:
            n_ij = np.random.uniform(-1, 1, 2)
            n_ij /= np.linalg.norm(n_ij)
            dist = 1e-8
        else:
            n_ij = d_vec / dist

        t_ij = np.array([-n_ij[1], n_ij[0]])
        r_ij = self.radius + r_j
        
        # g(x) function
        # overlap = r_ij - dist
        # g = max(overlap, 0.0)
        g = max(r_ij - dist, 0.0)

        # Exponential repulsive force
        f_exp = A * np.exp((r_ij - dist) / B) * n_ij

        # Pushing force
        f_push = k * g * n_ij

        # Sliding friction force
        dv_t = np.dot((v_j - self.vel), t_ij)
        f_slide = kappa * g * dv_t * t_ij

        return f_exp + f_push + f_slide

    @staticmethod
    def closest_point_on_segment(P, A, B):
    
        A = np.array(A, float)
        B = np.array(B, float)
        P = np.array(P, float)
        
        AB = B - A
        if np.allclose(AB, 0):
            return A
        t = np.dot(P - A, AB) / np.dot(AB, AB)
        t = np.clip(t, 0, 1)
        return A + t * AB