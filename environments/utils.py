import numpy as np

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

def path_intersection_in_time(p1, v1, p2, v2, dt, eps=1e-8):
    dp = p1 - p2
    dv = v1 - v2

    # If relative velocity is essentially zero
    if np.all(np.abs(dv) < eps):
        return None  # no relative motion → never meet

    # Compute t only where dv != 0
    mask = np.abs(dv) >= eps

    # If any dp != 0 in a dimension where dv == 0 → no solution
    if np.any(~mask & (np.abs(dp) >= eps)):
        return None

    # For dimensions with dv != 0, compute t candidates
    t_vals = -dp[mask] / dv[mask]

    # All t's must be equal → check variance
    # Much faster than looping over them
    if np.max(t_vals) - np.min(t_vals) > 1e-6:
        return None

    t = t_vals[0]

    return t if 0.0 <= t <= dt else None

    
    