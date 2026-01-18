import numpy as np
import math
from environments.agent import Agent

class BoidsAgent(Agent):
    def __init__(self, environment, config, start_pos=None, uid=None):
        if uid is None:
            uid = id(self)
        super().__init__(environment, uid)

        if start_pos is not None:
            self.pos = np.array(start_pos, dtype=float)
            self.prev_pos = self.pos.copy()
        else:
            self.prev_pos = self.pos.copy()

        self.config = config

        # --- PHYSICAL PARAMETERS ---
        self.max_speed = getattr(config, 'SPEED_LIMIT', 3.0)
        self.radius = getattr(config, 'AGENT_SIZE', 0.3)
        self.vision_radius = getattr(config, 'VISION_RADIUS', 80.0)
        self.tau = 0.1  # Instant acceleration
        self.mass = 1.0

        # --- FIX: ASSIGN DEFAULT TARGET ---
        # If the agent has no target, pick the first exit from the environment
        if self.target is None:
            if hasattr(self.env, 'exits') and len(self.env.exits) > 0:
                # Assign the first exit as the target
                self.target = self.env.exits[0]
            else:
                # Warn if no exits exist (agent will stay still)
                print(f"Warning: Agent {self.id} has no target and environment has no exits.")

        # --- WEIGHTS (Linked to Config) ---
        self.weights = {
            'sfm_driving': getattr(config, 'W_SEEK', 2.0),
            'sfm_agent': getattr(config, 'W_SEPARATE', 2.0),
            'sfm_wall': getattr(config, 'W_AVOID', 3.5),
            'boids_align': getattr(config, 'W_ALIGN', 0.5),
            'boids_cohere': getattr(config, 'W_COHERE', 0.5)
        }

    def update(self, dt, agent_snapshot):
        self.prev_pos[:] = self.pos

        # 1. Driving Force: Pulls to center of door
        f_driving = self.driving_force()

        # 2. Agent Repulsion
        f_repulsion = self.repulsive_force(agent_snapshot, A=1.5, B=0.1, k=500.0, kappa=0.0)

        # 3. Wall Repulsion
        walls = self.env.get_walls() if hasattr(self.env, 'get_walls') else []
        f_wall = self.obstacle_force(walls, A=2.0, B=0.1, k=800.0, kappa=0.0, r_i=self.radius * 0.2)

        # 4. Flocking
        neighbors = self._get_neighbors(agent_snapshot)
        f_align = self._compute_alignment(neighbors)
        f_cohere = self._compute_cohesion(neighbors)

        # 5. Anti-Stuck Jitter
        f_jitter = np.zeros(2)
        if np.linalg.norm(self.vel) < 0.2:
            f_jitter = (np.random.rand(2) - 0.5) * 5.0

        # Sum forces
        total_force = (
                f_driving * self.weights['sfm_driving'] +
                f_repulsion * self.weights['sfm_agent'] +
                f_wall * self.weights['sfm_wall'] +
                f_align * self.weights['boids_align'] +
                f_cohere * self.weights['boids_cohere'] +
                f_jitter
        )

        acc = total_force / self.mass
        self.vel = self.vel + acc * dt

        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
            self.vel = (self.vel / speed) * self.max_speed

        self.pos = self.pos + self.vel * dt

        self._check_and_resolve_collision()

    def driving_force(self):
        # FIX: Safety check for missing target
        if self.target is None:
            return np.zeros(2)

        target_array = np.array(self.target)

        if target_array.ndim == 1:
            # Target is a single point [x, y]
            direction = target_array - self.pos
        else:
            # Target is a segment (door) [[x1, y1], [x2, y2]]
            # We target the CENTER of the door segment
            try:
                A, B = target_array[0], target_array[1]
                door_center = (A + B) / 2.0
                direction = door_center - self.pos
            except IndexError:
                # Fallback if target format is weird
                return np.zeros(2)

        dist = np.linalg.norm(direction)
        if dist > 1e-8:
            direction = direction / dist
        else:
            direction = np.zeros(2)

        v_desired = direction * self.max_speed
        return (v_desired - self.vel) / self.tau

    def obstacle_force(self, walls, A=2.0, B=0.5, k=1.2e5, kappa=2.4e5, r_i=0.3):
        total = np.zeros(2)
        for wall in walls:
            if isinstance(wall, (list, tuple)) and len(wall) >= 2:
                wA = np.array(wall[0], float)
                wB = np.array(wall[1], float)
            else:
                continue

            seg = wB - wA
            seg_len_sq = np.dot(seg, seg)
            if seg_len_sq == 0:
                closest = wA
            else:
                t = np.dot(self.pos - wA, seg) / seg_len_sq
                t = np.clip(t, 0, 1)
                closest = wA + t * seg

            total += self._repulsion_from_point(
                p_j=closest, v_j=np.zeros(2), r_j=0.0,
                A=A, B=B, k=k, kappa=kappa, r_i=r_i
            )
        return total

    def _check_and_resolve_collision(self):
        if not hasattr(self.env, 'check_something_reached'): return
        wall_idx = self.env.check_something_reached(self.prev_pos, self.pos, "wall")
        if wall_idx is not None:
            wall = self.env.get_wall(wall_idx)
            wall_start, wall_end = np.array(wall[0]), np.array(wall[1])
            hit_point = self._get_wall_intersection(self.prev_pos, self.pos, wall_start, wall_end)

            wall_vec = wall_end - wall_start
            normal = np.array([-wall_vec[1], wall_vec[0]])
            normal = normal / (np.linalg.norm(normal) + 1e-8)
            if np.dot(normal, self.prev_pos - hit_point) < 0: normal = -normal

            self.pos = hit_point + (normal * 0.05)
            vel_dot_normal = np.dot(self.vel, normal)
            if vel_dot_normal < 0:
                self.vel -= normal * vel_dot_normal

    def _get_wall_intersection(self, p1, p2, w1, w2):
        xdiff = (p1[0] - p2[0], w1[0] - w2[0])
        ydiff = (p1[1] - p2[1], w1[1] - w2[1])

        def det(a, b): return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0: return p2
        d = (det(p1, p2), det(w1, w2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return np.array([x, y])

    def _get_neighbors(self, agents):
        neighbors = []
        for other in agents:
            if other is self: continue
            other_pos = other.get_position() if hasattr(other, 'get_position') else other.pos
            if np.linalg.norm(self.pos - other_pos) < self.vision_radius:
                neighbors.append(other)
        return neighbors

    def _compute_alignment(self, neighbors):
        if not neighbors: return np.zeros(2)
        avg_vel = np.mean([a.vel if hasattr(a, 'vel') else np.zeros(2) for a in neighbors], axis=0)
        norm = np.linalg.norm(avg_vel)
        if norm > 0:
            return ((avg_vel / norm * self.max_speed) - self.vel) / self.tau
        return np.zeros(2)

    def _compute_cohesion(self, neighbors):
        if not neighbors: return np.zeros(2)
        avg_pos = np.mean([a.get_position() if hasattr(a, 'get_position') else a.pos for a in neighbors], axis=0)
        direction = avg_pos - self.pos
        dist = np.linalg.norm(direction)
        if dist > 0:
            return ((direction / dist * self.max_speed) - self.vel) / self.tau
        return np.zeros(2)

    def get_smart_target(self):
        return self.target