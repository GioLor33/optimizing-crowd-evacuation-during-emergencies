import numpy as np
from environments.agent import Agent
from environments.utils import intersection_point
from boids_algorithm.boidsConfig import BoidsConfig

class BoidsAgent (Agent):
    def __init__(self, env_instance, uid, config: BoidsConfig):
        super().__init__(env_instance, uid)
        self.prev_pos = None
        self.vel = (np.random.rand(2) - 0.5) * 2
        self.acc = np.zeros(2, dtype=float)
        self.max_speed = config.SPEED_LIMIT
        self.max_force = config.FORCE_LIMIT
        
        self.agent_size = config.AGENT_SIZE
        self.vision_radius = config.VISION_RADIUS
        self.min_separation = config.MIN_SEPARATION
        self.wall_avoid_distance = config.WALL_AVOID_DISTANCE
        
        self.w_seek = config.W_SEEK
        self.w_avoid = config.W_AVOID
        self.w_separate = config.W_SEPARATE
        self.w_align = config.W_ALIGN
        self.w_cohere = config.W_COHERE
        
    def _limit_vector(self, vector, max_val):
        norm = np.linalg.norm(vector)
        if norm > max_val and norm > 0:
            return (vector / norm) * max_val
        return vector

    def _seek(self, target_pos):
        desired = target_pos - self.pos
        dist = np.linalg.norm(desired)
        if dist == 0: return np.zeros(2)

        desired = desired / dist
        if dist < 2:
            desired *= (dist / 2) * self.max_speed
        else:
            desired *= self.max_speed

        steering = desired - self.vel
        return self._limit_vector(steering, self.max_force)

    def _avoid_walls(self):
        # If not moving, nothing to avoid
        if np.linalg.norm(self.vel) == 0: return np.zeros(2)
        vel_versor = self.vel / np.linalg.norm(self.vel)
        ahead_pos = self.pos + vel_versor * self.wall_avoid_distance
        wall = self.env.check_something_reached(self.pos, ahead_pos, "wall")
        if wall is not None:
            wall = self.env.get_wall(wall)
            p1 = np.array(wall[0])
            p2 = np.array(wall[1])
            wall_vector = p2 - p1
            normal = np.array([-wall_vector[1], wall_vector[0]])
            norm_length = np.linalg.norm(normal)
            if norm_length > 0:
                normal = normal / norm_length  # Normalize
            to_agent = self.pos - p1
            if np.dot(to_agent, normal) < 0:
                normal = -normal
            overshoot = ahead_pos - self.pos
            distance_to_wall = np.linalg.norm(overshoot)
            force_magnitude = self.max_force * 2

            return normal * force_magnitude

        return np.zeros(2)

    def _flock(self, agents_snapshot):
        sep = np.zeros(2)
        ali = np.zeros(2)
        coh = np.zeros(2)
        count = 0

        for other in agents_snapshot:
            if other.id == self.id: continue

            diff = self.pos - other.pos
            dist = np.linalg.norm(diff)

            if 0 < dist < self.vision_radius:
                if dist < self.min_separation:
                    diff /= dist
                    sep += diff
                ali += other.vel
                coh += other.pos
                count += 1

        if np.linalg.norm(sep) > 0:
            sep = (sep / np.linalg.norm(sep)) * self.max_speed
            sep = self._limit_vector(sep - self.vel, self.max_force)

        if count > 0:
            ali /= count
            if np.linalg.norm(ali) > 0:
                ali = (ali / np.linalg.norm(ali)) * self.max_speed
                ali = self._limit_vector(ali - self.vel, self.max_force)
            coh /= count
            coh = self._seek(coh)

        return sep, ali, coh

    def _check_and_resolve_collision(self):
        wall = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
        if wall is not None:
            wall = self.env.get_wall(wall)
            wall_center = intersection_point(self.prev_pos, self.pos, wall[0], wall[1])
            penetration_vector = self.pos - wall_center
            safe_distance = self.agent_size + 0.5
            current_distance = np.linalg.norm(penetration_vector)
            if current_distance < safe_distance:
                if current_distance == 0:
                    push_dir = -self.vel / np.linalg.norm(self.vel)
                    overlap = safe_distance
                else:
                    push_dir = penetration_vector / current_distance
                    overlap = safe_distance - current_distance
                self.pos += push_dir * overlap
                vel_dot_normal = np.dot(self.vel, push_dir)
                if vel_dot_normal < 0:
                    self.vel -= push_dir * vel_dot_normal
                return True
        return False

    def update(self, agents_snapshot , dt):
        self.acc *= 0.0
        self.prev_pos = self.pos.copy()

        f_seek = self._seek(self.target) * self.w_seek
        f_avoid = self._avoid_walls() * self.w_avoid
        f_sep, f_ali, f_coh = self._flock(agents_snapshot)

        if np.linalg.norm(f_avoid) > 0.1:
            self.acc += f_avoid
            self.acc += f_sep * self.w_separate #* 0.8
        else:
            self.acc += f_seek
            self.acc += (f_sep * self.w_separate)
            self.acc += (f_ali * self.w_align)
            self.acc += (f_coh * self.w_cohere)

        self.vel += self.acc * dt
        self.vel = self._limit_vector(self.vel, self.max_speed)
        self.pos += self.vel * dt
        self._check_and_resolve_collision()
        
        #Here we are scaling position such that agents stay within bounds
        # w, h = self.env.get_dimensions()
        # if self.pos[0] < self.agent_size: 
        #     self.pos[0] = self.agent_size
        #     self.vel[0] *= -1
        # if self.pos[0] > w - self.agent_size:
        #     self.pos[0] = w - self.agent_size
        #     self.vel[0] *= -1
        # if self.pos[1] < self.agent_size: 
        #     self.pos[1] = self.agent_size
        #     self.vel[1] *= -1
        # if self.pos[1] > h - self.agent_size: 
        #     self.pos[1] = h - self.agent_size
        #     self.vel[1] *= -1