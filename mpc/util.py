import numpy as np

def find_closest_point(points, ref_point):
    return np.argmin(np.linalg.norm(points - ref_point, axis=1))

def extract_next_path_points(path_points, pos, N):
    closest_point = find_closest_point(path_points, pos)
    n_points = path_points.shape[0]
    next_path_points = list(path_points[closest_point+1:min(closest_point+N+1, n_points)]) + \
        [path_points[-1]] * int(max(0, closest_point + N + 1 - n_points))
    return next_path_points