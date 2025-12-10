import numpy as np
import math


class BoidsAgent:
    def __init__(self, environment, config, start_pos=None):
        self.env = environment
        self.id = id(self)

        # 1. Physics & State
        start = start_pos if start_pos is not None else self.env.get_random_spawn()
        self.pos = np.array(start, dtype=float)
        self.prev_pos = self.pos.copy()
        self.vel = np.random.uniform(-1, 1, 2)
        self.acc = np.zeros(2)

        # 2. Config (Condensed)
        self.radius = getattr(config, 'agent_radius', 5)
        self.base_speed = getattr(config, 'max_speed', 2.0)
        self.base_force = getattr(config, 'max_force', 0.1)

        # Flocking & Vision Config
        self.vision_radius = getattr(config, 'vision_radius', 100)
        self.min_separation = getattr(config, 'min_separation', 25)
        self.wall_avoid_dist = getattr(config, 'wall_avoid_dist', 20)
        self.exits = [(np.array(p1), np.array(p2)) for p1, p2 in self.env.get_safety_exits()]

        # Weights map
        self.weights = {
            'seek': getattr(config, 'w_seek', 1.0),
            'avoid': getattr(config, 'w_avoid', 1.5),
            'sep': getattr(config, 'w_separate', 1.5),
            'ali': getattr(config, 'w_align', 1.0),
            'coh': getattr(config, 'w_cohere', 1.0)
        }

        # Dynamic State
        self.cur_speed = self.base_speed
        self.cur_force = self.base_force

        # 3. Vision Rays (Pre-calculated)
        angles = [i * (math.pi / 12) for i in range(1, 13)]
        self.rays = []
        for a in angles:
            c, s = math.cos(a), math.sin(a)
            self.rays.extend([np.array([[c, -s], [s, c]]), np.array([[c, s], [-s, c]])])

    def update(self, dt, agents_snapshot=None):
        self.prev_pos[:] = self.pos

        # 1. Decision Making
        target = self.vision(self.get_smart_target())
        dist_sq = np.sum((target - self.pos) ** 2)

        # 2. Sprint Logic
        if dist_sq < 16.0:
            self.cur_speed, self.cur_force = self.base_speed * 1.5, self.base_force * 3.0
            f_avoid = np.zeros(2)
        else:
            self.cur_speed, self.cur_force = self.base_speed, self.base_force
            f_avoid = self.avoid_walls() * self.weights['avoid']

        # 3. Calculate Forces
        self.acc += self.seek(target) * self.weights['seek']
        self.acc += f_avoid

        if agents_snapshot:
            sep, ali, coh = self._flock(agents_snapshot)
            self.acc += (sep * self.weights['sep']) + (ali * self.weights['ali']) + (coh * self.weights['coh'])

        # 4. Physics Step
        self.vel += self.acc
        speed = np.linalg.norm(self.vel)
        if speed > self.cur_speed:
            self.vel = (self.vel / speed) * self.cur_speed
        self.pos += self.vel * dt
        self._check_and_resolve_collision()
        self.acc.fill(0)

    def vision(self, target):
        to_target = target - self.pos
        dist = np.linalg.norm(to_target)
        if dist == 0: return target
        dir_vec = to_target / dist
        #put vision radius limit here
        check_pos = self.pos + (dir_vec * min(dist, self.vision_radius))
        if not self.env.check_something_reached(self.pos, check_pos, "wall"):
            return target
        for rot in self.rays:
            look_ahead = self.pos + ((rot @ dir_vec) * 4.0)
            if not self.env.check_something_reached(self.pos, look_ahead, "wall"):
                return look_ahead
        return target

    def _check_and_resolve_collision(self):
        wall = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
        if wall is not None:
            wall_start, wall_end = np.array(wall[0]), np.array(wall[1])
            wall_center = self._get_wall_intersection(self.prev_pos, self.pos, wall_start, wall_end)
            penetration_vector = self.pos - wall_center
            safe_distance = self.radius + 0.5  # mapped from agent_size
            current_distance = np.linalg.norm(penetration_vector)
            if current_distance < safe_distance:
                if current_distance == 0:
                    if np.linalg.norm(self.vel) > 0:
                        push_dir = -self.vel / np.linalg.norm(self.vel)
                    else:
                        push_dir = np.array([1.0, 0.0])
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

    def _get_wall_intersection(self, p1, p2, w1, w2):
        xdiff = (p1[0] - p2[0], w1[0] - w2[0])
        ydiff = (p1[1] - p2[1], w1[1] - w2[1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return p2

        d = (det(p1, p2), det(w1, w2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return np.array([x, y])

    def _flock(self, agents_snapshot):
        sep = np.zeros(2)
        ali = np.zeros(2)
        coh = np.zeros(2)
        count = 0

        for other in agents_snapshot:
            if getattr(other, 'id', id(other)) == self.id: continue

            other_pos = other.get_position() if hasattr(other, 'get_position') else other.pos
            other_vel = other.get_velocity() if hasattr(other, 'get_velocity') else other.vel

            diff = self.pos - other_pos
            dist = np.linalg.norm(diff)

            if 0 < dist < self.vision_radius:
                if dist < self.min_separation:
                    diff /= dist
                    sep += diff
                ali += other_vel
                coh += other_pos
                count += 1

        if np.linalg.norm(sep) > 0:
            sep = (sep / np.linalg.norm(sep)) * self.cur_speed
            sep = self._limit_vector(sep - self.vel, self.cur_force)

        if count > 0:
            ali /= count
            if np.linalg.norm(ali) > 0:
                ali = (ali / np.linalg.norm(ali)) * self.cur_speed
                ali = self._limit_vector(ali - self.vel, self.cur_force)
            coh /= count
            coh = self.seek(coh)

        return sep, ali, coh

    def avoid_walls(self):
        steer = np.zeros(2)
        count = 0
        for p1, p2 in self.env.get_walls():
            p1, p2 = np.array(p1), np.array(p2)
            v_wall = p2 - p1
            len_sq = np.dot(v_wall, v_wall) or 1.0

            t = np.clip(np.dot(self.pos - p1, v_wall) / len_sq, 0, 1)
            closest = p1 + (v_wall * t)
            diff = self.pos - closest
            dist_sq = np.dot(diff, diff)

            if 0 < dist_sq < 16.0:  # 4.0 squared
                steer += diff / math.sqrt(dist_sq)
                count += 1

        if count > 0:
            steer /= count
            # Use _limit_vector instead of _limit_force
            steer = self._limit_vector(self._set_mag(steer, self.cur_speed) - self.vel, self.cur_force)
        return steer

    def seek(self, target):
        desired = target - self.pos
        if np.dot(desired, desired) > 0:
            steer = self._set_mag(desired, self.cur_speed) - self.vel
            # Use _limit_vector instead of _limit_force
            return self._limit_vector(steer, self.cur_force)
        return np.zeros(2)

    def get_smart_target(self):
        if not self.exits: return self.pos
        best, min_d = self.pos, float('inf')
        for p1, p2 in self.exits:
            v_ex = p2 - p1
            t = np.clip(np.dot(self.pos - p1, v_ex) / (np.dot(v_ex, v_ex) or 1), 0.1, 0.9)
            pt = p1 + v_ex * t
            d = np.sum((self.pos - pt) ** 2)
            if d < min_d: best, min_d = pt, d
        return best
    def _set_mag(self, vec, mag):
        sq = np.dot(vec, vec)
        return vec * (mag / math.sqrt(sq)) if sq > 0 else vec

    def _limit_vector(self, vector, max_val):
        norm = np.linalg.norm(vector)
        if norm > max_val and norm > 0:
            return (vector / norm) * max_val
        return vector

    # --- External Getters ---
    def get_position(self):
        return self.pos

    def get_velocity(self):
        return self.vel

    def get_radius(self):
        return self.radius

    def get_smart_target_visual(self):
        return self.vision(self.get_smart_target())