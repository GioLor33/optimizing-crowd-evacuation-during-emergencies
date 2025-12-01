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
    