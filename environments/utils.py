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

def wall_intersection_point(pos, ahead_pos, wall_start, wall_end):
    x0 = pos[0]
    y0 = pos[1]
    x1 = ahead_pos[0]
    y1 = ahead_pos[1]
    x2 = wall_start[0]
    y2 = wall_start[1]
    x3 = wall_end[0]
    y3 = wall_end[1]
    
    # det := np.ling.det()

    l1 = np.array([
            [x0-x2, x2-x3],
            [y0-y2, y2-y3]
        ])
    l2 = np.array([
            [x0-x1, x2-x3],
            [y0-y1, y2-y3]
        ])
    t = np.linalg.det(l1) / np.linalg.det(l2)
    intersection_x = x0 + t * (x1 - x0)
    intersection_y = y0 + t * (y1 - y0)
    
    return np.array([intersection_x, intersection_y])
    