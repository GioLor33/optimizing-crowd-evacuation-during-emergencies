import numpy as np
from environments.agent import Agent
from environments.utils import wall_intersection_point

# TODO: read this from config file
AGENT_SIZE = 5.0
VISION_RADIUS = 7
MIN_SEPARATION = 3 * AGENT_SIZE
WALL_AVOID_DISTANCE = 10
SPEED_LIMIT = 2.5
FORCE_LIMIT = 0.3

W_SEEK = 1.0
W_AVOID = 3.5
W_SEPARATE = 2.0
W_ALIGN = 0.5
W_COHERE = 0.5

class BoidsAgent (Agent):
    def __init__(self, env_instance, uid):
        super().__init__(env_instance, uid)
        self.prev_pos = None
        self.vel = (np.random.rand(2) - 0.5) * 2
        self.acc = np.zeros(2, dtype=float)
        self.max_speed = SPEED_LIMIT
        self.max_force = FORCE_LIMIT

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

    # def _avoid_walls(self):
    #     if np.linalg.norm(self.vel) == 0: return np.zeros(2)
    #
    #     vel_versor = self.vel / np.linalg.norm(self.vel)
    #     ahead_pos = self.pos + vel_versor * WALL_AVOID_DISTANCE
    #
    #     wall = self.env.check_something_reached(self.pos, ahead_pos, "wall")
    #     if wall is not None:
    #         # intersection_with_wall = wall_intersection_point(self.pos, ahead_pos, wall[0], wall[1])
    #         # flee_dir = ahead_pos - intersection_with_wall
    #         # if np.linalg.norm(flee_dir) > 0:
    #         #     flee_dir = (flee_dir / np.linalg.norm(flee_dir)) * self.max_speed
    #         # steering = flee_dir - self.vel
    #
    #         steering = vel_versor * (self.max_speed - np.linalg.norm(self.vel))
    #
    #         return self._limit_vector(steering, self.max_force * 2)
    #
    #     return np.zeros(2)

    def _avoid_walls(self):
        # If not moving, nothing to avoid
        if np.linalg.norm(self.vel) == 0: return np.zeros(2)
        vel_versor = self.vel / np.linalg.norm(self.vel)
        ahead_pos = self.pos + vel_versor * WALL_AVOID_DISTANCE
        wall = self.env.check_something_reached(self.pos, ahead_pos, "wall")
        if wall is not None:
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

            if 0 < dist < VISION_RADIUS:
                if dist < MIN_SEPARATION:
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

    # def _check_and_resolve_collision(self):
    #     wall = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
    #     if wall is not None:
    #         wall_center = wall_intersection_point(self.prev_pos, self.pos, wall[0], wall[1])
    #         penetration_vector = self.pos - wall_center
    #         min_distance_to_exit = 2.0 + AGENT_SIZE
    #         current_distance = np.linalg.norm(penetration_vector)
    #
    #         if current_distance < min_distance_to_exit:
    #             overlap = min_distance_to_exit - current_distance
    #
    #             if current_distance > 0:
    #                 push_direction = penetration_vector / current_distance
    #                 self.pos += push_direction * overlap
    #                 self.vel = self.vel - 2 * np.dot(self.vel, push_direction) * push_direction
    #                 # self.vel *= 0.8
    #                 return True
    #     return False

    def _check_and_resolve_collision(self):
        wall = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
        if wall is not None:
            wall_center = wall_intersection_point(self.prev_pos, self.pos, wall[0], wall[1])
            penetration_vector = self.pos - wall_center
            safe_distance = AGENT_SIZE + 0.5
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

        f_seek = self._seek(self.target) * W_SEEK
        f_avoid = self._avoid_walls() * W_AVOID
        f_sep, f_ali, f_coh = self._flock(agents_snapshot)

        if np.linalg.norm(f_avoid) > 0.1:
            self.acc += f_avoid
            self.acc += f_sep * W_SEPARATE #* 0.8
        else:
            self.acc += f_seek
            self.acc += (f_sep * W_SEPARATE)
            self.acc += (f_ali * W_ALIGN)
            self.acc += (f_coh * W_COHERE)

        self.vel += self.acc * dt
        self.vel = self._limit_vector(self.vel, self.max_speed)
        self.pos += self.vel * dt
        self._check_and_resolve_collision()
        
        #Here we are scaling position such that agents stay within bounds
        # w, h = self.env.get_dimensions()
        # if self.pos[0] < AGENT_SIZE: 
        #     self.pos[0] = AGENT_SIZE
        #     self.vel[0] *= -1
        # if self.pos[0] > w - AGENT_SIZE:
        #     self.pos[0] = w - AGENT_SIZE
        #     self.vel[0] *= -1
        # if self.pos[1] < AGENT_SIZE: 
        #     self.pos[1] = AGENT_SIZE
        #     self.vel[1] *= -1
        # if self.pos[1] > h - AGENT_SIZE: 
        #     self.pos[1] = h - AGENT_SIZE
        #     self.vel[1] *= -1