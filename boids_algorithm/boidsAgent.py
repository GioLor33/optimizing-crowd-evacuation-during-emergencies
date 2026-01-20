import numpy as np
import math
from environments.agent import Agent


class BoidsAgent(Agent):
    def __init__(self, environment, config, start_pos=None):
        super().__init__(environment, id(self))
        start = start_pos if start_pos is not None else self.env.get_random_spawn()
        self.pos = np.array(start, dtype=float)
        self.prev_pos = self.pos.copy()
        self.vel = np.random.uniform(-1, 1, 2)
        self.acc = np.zeros(2)
        self.radius = getattr(config, 'agent_radius', 0.3)
        self.base_speed = getattr(config, 'max_speed', 8.0)
        self.mass = getattr(config, 'mass', 75.0)
        self.tau = getattr(config, 'tau', 0.5)
        self.vision_radius = getattr(config, 'vision_radius', 10.0)
        self.min_separation = getattr(config, 'min_separation', 1.0)
        self.exits = [(np.array(p1), np.array(p2)) for p1, p2 in self.env.get_safety_exits()]

        self.weights = {
                    'ali': getattr(config, 'w_align', 2.0),
                    'coh': getattr(config, 'w_cohere', 2.0),
                    'sfm_walls': getattr(config, 'sfm_walls', 3.5),   # Was hardcoded 3.5
                    'sfm_agents': getattr(config, 'sfm_agents', 0.5)  # Was hardcoded 0.5
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


import numpy as np
import math
from environments.agent import Agent

#
# class BoidsAgent(Agent):
#
#     def __init__(self, environment, config, start_pos=None):
#         # Initialize parent (sets id, env, random pos, vel, etc.)
#         super().__init__(environment, id(self))
#         if start_pos is not None:
#             self.pos = np.array(start_pos, dtype=float)
#
#         self.prev_pos = self.pos.copy()
#         self.acc = np.zeros(2)  # Agent does not init acc
#
#         self.radius = getattr(config, 'agent_radius', 0.2)
#         self.base_speed = getattr(config, 'max_speed', 2.0)
#         self.base_force = getattr(config, 'max_force', 0.1)
#
#         self.vision_radius = getattr(config, 'vision_radius', 80)
#         self.min_separation = getattr(config, 'min_separation', 25)
#         self.wall_avoid_dist = getattr(config, 'wall_avoid_dist', 0.5)
#
#         self.exits = [(np.array(p1), np.array(p2)) for p1, p2 in self.env.get_safety_exits()]
#
#         self.weights = {
#             'seek': getattr(config, 'w_seek', 3.0863),
#             'avoid': getattr(config, 'w_avoid', 4.0519),
#             'sep': getattr(config, 'w_separate', 0.8963),
#             'ali': getattr(config, 'w_align', 3.0000),
#             'coh': getattr(config, 'w_cohere', 2.2328)
#         }
#
#         self.cur_speed = self.base_speed
#         self.cur_force = self.base_force
#         self.f_desired = np.zeros(2)
#         self.f_agents = np.zeros(2)
#         self.f_walls = np.zeros(2)
#         angles = [i * (math.pi / 12) for i in range(1, 13)]
#         self.rays = []
#         for a in angles:
#             c, s = math.cos(a), math.sin(a)
#             self.rays.extend([
#                 np.array([[c, -s], [s, c]]),
#                 np.array([[c, s], [-s, c]])
#             ])
#
#     def update(self, dt, agents_snapshot=None):
#         self.prev_pos[:] = self.pos
#         self.acc.fill(0)
#         self.f_desired.fill(0)
#         self.f_agents.fill(0)
#         self.f_walls.fill(0)
#         target = self.vision(self.get_smart_target())
#         dist_sq = np.sum((target - self.pos) ** 2)
#         if dist_sq < 16.0:
#             self.cur_speed = self.base_speed * 1.5
#             self.cur_force = self.base_force * 3.0
#         else:
#             self.cur_speed = self.base_speed
#             self.cur_force = self.base_force
#         self.f_desired = self.seek(target) * self.weights['seek']
#         self.f_walls = self.avoid_walls() * self.weights['avoid']
#         if agents_snapshot:
#             sep, ali, coh = self._flock(agents_snapshot)
#             self.f_agents = (
#                     sep * self.weights['sep'] +
#                     ali * self.weights['ali'] +
#                     coh * self.weights['coh']
#             )
#         self.acc += self.f_desired
#         self.acc += self.f_walls
#         self.acc += self.f_agents
#         self.vel += self.acc
#         speed = np.linalg.norm(self.vel)
#         if speed > self.cur_speed:
#             self.vel = (self.vel / speed) * self.cur_speed
#
#         self.pos += self.vel * dt
#         self._check_and_resolve_collision()
#
#     def vision(self, target):
#         to_target = target - self.pos
#         dist = np.linalg.norm(to_target)
#         if dist == 0:
#             return target
#
#         dir_vec = to_target / dist
#         check_pos = self.pos + (dir_vec * min(dist, self.vision_radius))
#
#         if not self.env.check_something_reached(self.pos, check_pos, "wall"):
#             return target
#
#         for rot in self.rays:
#             look_ahead = self.pos + ((rot @ dir_vec) * 4.0)
#             if not self.env.check_something_reached(self.pos, look_ahead, "wall"):
#                 return look_ahead
#
#         return target
#
#     def _flock(self, agents_snapshot):
#         sep = np.zeros(2)
#         ali = np.zeros(2)
#         coh = np.zeros(2)
#         count = 0
#
#         for other in agents_snapshot:
#             if getattr(other, 'id', id(other)) == self.id:
#                 continue
#
#             other_pos = other.get_position()
#             if hasattr(other, 'get_velocity'):
#                 other_vel = other.get_velocity()
#             else:
#                 other_vel = other.vel
#
#             diff = self.pos - other_pos
#             dist = np.linalg.norm(diff)
#
#             if 0 < dist < self.vision_radius:
#                 if dist < self.min_separation:
#                     sep += diff / dist
#                 ali += other_vel
#                 coh += other_pos
#                 count += 1
#
#         if np.linalg.norm(sep) > 0:
#             sep = self._limit_vector(
#                 self._set_mag(sep, self.cur_speed) - self.vel,
#                 self.cur_force
#             )
#
#         if count > 0:
#             ali /= count
#             if np.linalg.norm(ali) > 0:
#                 ali = self._limit_vector(
#                     self._set_mag(ali, self.cur_speed) - self.vel,
#                     self.cur_force
#                 )
#             coh /= count
#             coh = self.seek(coh)
#
#         return sep, ali, coh
#
#     def avoid_walls(self):
#         steer = np.zeros(2)
#         count = 0
#         avoid_radius = self.wall_avoid_dist
#         avoid_radius_sq = avoid_radius ** 2
#         for p1, p2 in self.env.get_walls():
#             p1, p2 = np.array(p1), np.array(p2)
#             closest = self.closest_point_on_segment(self.pos, p1, p2)
#             diff = self.pos - closest
#             dist_sq = np.dot(diff, diff)
#             if 0 < dist_sq < avoid_radius_sq:
#                 dist = math.sqrt(dist_sq)
#                 weight = (avoid_radius - dist) / dist
#                 steer += (diff / dist) * weight
#                 count += 1
#
#         if count > 0:
#             steer /= count
#             steer = self._limit_vector(
#                 self._set_mag(steer, self.cur_speed) - self.vel,
#                 self.cur_force * 2
#             )
#
#         return steer
#
#     def seek(self, target):
#         desired = target - self.pos
#         if np.dot(desired, desired) > 0:
#             steer = self._set_mag(desired, self.cur_speed) - self.vel
#             return self._limit_vector(steer, self.cur_force)
#         return np.zeros(2)
#
#     def get_smart_target(self):
#         if not self.exits:
#             return self.pos
#
#         best = self.pos
#         min_d = float('inf')
#
#         for p1, p2 in self.exits:
#             v = p2 - p1
#             t = np.clip(np.dot(self.pos - p1, v) / (np.dot(v, v) or 1), 0.1, 0.9)
#             pt = p1 + v * t
#             d = np.sum((self.pos - pt) ** 2)
#             if d < min_d:
#                 best, min_d = pt, d
#
#         return best
#     def _check_and_resolve_collision(self):
#         hit_index = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
#         if hit_index is None:
#             return False
#         w1, w2 = self.env.get_wall(hit_index)
#         w1, w2 = np.array(w1), np.array(w2)
#         closest = self.closest_point_on_segment(self.pos, w1, w2)
#         wall_vec = w2 - w1
#         wall_len = np.linalg.norm(wall_vec)
#         if wall_len == 0:
#             return False
#         wall_dir = wall_vec / wall_len
#         wall_normal = np.array([-wall_dir[1], wall_dir[0]])
#         if np.dot(self.prev_pos - closest, wall_normal) < 0:
#             wall_normal = -wall_normal
#         self.pos = closest + wall_normal * (self.radius + 0.1)
#         vn = np.dot(self.vel, wall_normal)
#         if vn < 0:
#             self.vel -= wall_normal * vn
#
#         return True
#
#     def _set_mag(self, vec, mag):
#         n = np.linalg.norm(vec)
#         return vec * (mag / n) if n > 0 else vec
#
#     def _limit_vector(self, vec, max_val):
#         n = np.linalg.norm(vec)
#         return (vec / n) * max_val if n > max_val and n > 0 else vec
#     def get_velocity(self):
#         return self.vel
#     def get_radius(self):
#         return self.radius
#     def get_smart_target_visual(self):
#         return self.vision(self.get_smart_target())
