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
        
        # SFM parameters
        self.A = 2.0
        self.B = 0.08
        self.k = 1.2e5
        self.kappa = 2.4e5
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
    
    def update(self, agent_snapshot, env, dt):
       
        if self.target is None:
            raise ValueError("Agent " + str(self.id) + " has no target assigned. If no specific target is needed, set target to global target.")

        f_desired = self.driving_force()

        f_agents = self.repulsive_force(
            agent_snapshot
        )

        f_walls = self.obstacle_force(
            env.get_walls()
        )

        self.f_desired = f_desired
        self.f_walls = f_walls
        self.f_agents = f_agents

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
            direction = np.zeros(2)
        else:
            direction /= norm

        v_des = direction * self.max_speed
        return (v_des - self.vel) / self.tau


    def repulsive_force(self, agents):
        total = np.zeros(2)
        for other in agents:
            if other is self:
                continue
            r_j = other.radius if hasattr(other, "radius") else self.radius
            total += self._repulsion_from_point(
                p_j=other.pos,
                v_j=other.vel,
                r_j=r_j
            )
        return total


    def obstacle_force(self, walls):
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
                r_j = 0.0
            )

        return total


    def _repulsion_from_point(self, p_j, v_j, r_j, r_i=None):
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

        g = max(r_ij - dist, 0.0)

        # Exponential repulsive force
        f_exp = self.A * np.exp((r_ij - dist) / self.B) * n_ij

        # Pushing force
        f_push = self.k * g * n_ij

        # Sliding friction force
        dv_t = np.dot((v_j - self.vel), t_ij)
        f_slide = self.kappa * g * dv_t * t_ij

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