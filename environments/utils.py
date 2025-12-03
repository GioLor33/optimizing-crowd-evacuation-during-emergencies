import numpy as np
import random

def segments_intersect(A, B, C, D):
    def orient(p, q, r):
        return np.cross(q - p, r - p)

    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    D = np.array(D)

    o1 = orient(A, B, C)
    o2 = orient(A, B, D)
    o3 = orient(C, D, A)
    o4 = orient(C, D, B)

    # General case
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True

    return False

def intersection_point(pos_1_start, pos_1_end, pos_2_start, pos_2_end):
    x0 = pos_1_start[0]
    y0 = pos_1_start[1]
    x1 = pos_1_end[0]
    y1 = pos_1_end[1]
    x2 = pos_2_start[0]
    y2 = pos_2_start[1]
    x3 = pos_2_end[0]
    y3 = pos_2_end[1]
    
    # det := np.ling.det()

    l1 = np.array([
            [x0-x2, x2-x3],
            [y0-y2, y2-y3]
        ])
    l2 = np.array([
            [x0-x1, x2-x3],
            [y0-y1, y2-y3]
        ])
    
    # Lines are parallel or coincident
    # TODO: should we handle coincident lines differently?
    if np.linalg.det(l2) == 0:
        return None  
    
    t = np.linalg.det(l1) / np.linalg.det(l2)
    intersection_x = x0 + t * (x1 - x0)
    intersection_y = y0 + t * (y1 - y0)
    
    return np.array([intersection_x, intersection_y])

def path_intersection_in_time(p1, v1, p2, v2, dt):
    dp = p1 - p2
    dv = v1 - v2
    
    # If relative velocity is zero: no motion toward each other
    if np.allclose(dv, 0):
        return None  

    # Compute t for each component
    t_values = []
    for i in range(2):
        if abs(dv[i]) < 1e-8:
            # No solution if dp[i] != 0
            if abs(dp[i]) > 1e-8:
                return None  
            # This dimension gives no constraint (parallel)
        else:
            t_values.append(dp[i] / dv[i])

    # All non-parallel dimensions must agree on t
    if len(t_values) == 0:
        return None
    t = t_values[0]
    for ti in t_values[1:]:
        if abs(t - ti) > 1e-6:
            return None  # no consistent solution

    return t if t >= 0 and t <= dt else None
    
    