import numpy as np
import math
from environments.agent import Agent


class BoidsAgent(Agent):
    def __init__(self, environment, config, start_pos=None):
        super().__init__(environment, id(self))
        start = start_pos if start_pos is not None else self.env.get_random_spawn()
        self.sfm_A = 10.0
        self.sfm_B = 0.8
        self.sfm_k = 500.0
        self.sfm_kappa = 1000.0
        self.mass = 80.0
        self.pos = np.array(start, dtype=float)
        self.prev_pos = self.pos.copy()
        self.vel = np.random.uniform(-1, 1, 2)
        self.acc = np.zeros(2)
        self.radius = getattr(config, 'agent_radius', 0.3)
        self.base_speed = getattr(config, 'max_speed', 2.0)
        self.mass = getattr(config, 'mass', 1.0)
        self.tau = getattr(config, 'tau', 0.5)
        self.vision_radius = getattr(config, 'vision_radius', 10.0)
        self.min_separation = getattr(config, 'min_separation', 1.0)
        self.exits = [(np.array(p1), np.array(p2)) for p1, p2 in self.env.get_safety_exits()]

        self.weights = {
            'ali': getattr(config, 'w_align', 1.0),
            'coh': getattr(config, 'w_cohere', 1.0),
            'sfm_walls': 1.0,
            'sfm_agents': 1.0
        }
        self.cur_speed = self.base_speed
        angles = [i * (math.pi / 12) for i in range(1, 13)]
        self.rays = []
        for a in angles:
            c, s = math.cos(a), math.sin(a)
            self.rays.extend([np.array([[c, -s], [s, c]]), np.array([[c, s], [-s, c]])])

    def update(self, dt, agents_snapshot=None):
        self.prev_pos[:] = self.pos
        visual_target = self.vision(self.get_smart_target())
        self.target = visual_target
        dist_sq = np.sum((visual_target - self.pos) ** 2)
        if dist_sq < 16.0:
            self.cur_speed = self.base_speed * 1.5
        else:
            self.cur_speed = self.base_speed
        self.max_speed = self.cur_speed
        f_drive = self.driving_force()
        f_walls = self.obstacle_force(self.env.get_walls()) * self.weights['sfm_walls']
        f_sfm_agents = self.repulsive_force(agents_snapshot) * self.weights['sfm_agents']
        ali, coh = self._get_boids_vectors(agents_snapshot)
        f_flock = (ali * self.weights['ali']) + (coh * self.weights['coh'])
        total_force = f_drive + f_walls + f_sfm_agents + f_flock
        self.acc = total_force / self.mass
        self.vel += self.acc * dt
        speed = np.linalg.norm(self.vel)
        if speed > self.cur_speed:
            self.vel = (self.vel / speed) * self.cur_speed

        self.pos += self.vel * dt

    def _get_boids_vectors(self, agents_snapshot):
        ali = np.zeros(2)
        coh = np.zeros(2)
        count = 0

        for other in agents_snapshot:
            if getattr(other, 'id', id(other)) == self.id: continue
            other_pos = other.get_position()
            other_vel = other.get_velocity()
            dist = np.linalg.norm(self.pos - other_pos)

            if 0 < dist < self.vision_radius:
                ali += other_vel
                coh += other_pos
                count += 1
        if count > 0:
            ali /= count
            if np.linalg.norm(ali) > 0:
                ali = (ali / np.linalg.norm(ali)) * self.cur_speed
                ali -= self.vel
            coh /= count
            desired = coh - self.pos
            if np.linalg.norm(desired) > 0:
                desired = (desired / np.linalg.norm(desired)) * self.cur_speed
                coh = desired - self.vel
            else:
                coh = np.zeros(2)

        return ali, coh

    def vision(self, target):
        to_target = target - self.pos
        dist = np.linalg.norm(to_target)
        if dist == 0: return target
        dir_vec = to_target / dist
        check_pos = self.pos + (dir_vec * min(dist, self.vision_radius))
        if not self.env.check_something_reached(self.pos, check_pos, "wall"):
            return target
        for rot in self.rays:
            look_ahead = self.pos + ((rot @ dir_vec) * 4.0)
            if not self.env.check_something_reached(self.pos, look_ahead, "wall"):
                return look_ahead
        return target

    def get_smart_target(self):
        if not self.exits: return self.pos
        best, min_d = self.pos, float('inf')
        for p1, p2 in self.exits:
            v_ex = p2 - p1
            len_sq = np.dot(v_ex, v_ex) or 1.0
            t = np.clip(np.dot(self.pos - p1, v_ex) / len_sq, 0.1, 0.9)
            pt = p1 + v_ex * t
            d = np.sum((self.pos - pt) ** 2)
            if d < min_d: best, min_d = pt, d
        return best


    def get_velocity(self):
        return self.vel

    def get_position(self):
        return self.pos

    def get_smart_target_visual(self):
        return self.vision(self.get_smart_target())