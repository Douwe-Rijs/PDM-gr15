import numpy as np
import heapq

# =============================
# 3D AABB Obstacle
# =============================
from check_point import is_point_in_obstacle, draw_debug_point

class AABB:
    def __init__(self, min_corner, max_corner):
        self.min = np.array(min_corner)
        self.max = np.array(max_corner)

    def contains(self, p):
        return np.all(p >= self.min) and np.all(p <= self.max)

# =============================
# Collision Checking (straight line)
# =============================

def line_collides(p0, p1, obstacles, steps=40, padding = 0.2):
    """Check the straight line segment p0→p1 for collisions."""
    for i in range(steps + 1):
        t = i / steps
        p = p0 * (1 - t) + p1 * t
        if is_point_in_obstacle(p, obstacles, padding):
            return True
    return False

# =============================
# Node (3D RRT)
# =============================
class Node:
    def __init__(self, pos, parent=None):
        self.pos = np.array(pos, dtype=float)
        self.parent = parent

# =============================
# Nearest Node (position only)
# =============================
def nearest(nodes, sample):
    best = None
    best_d = float("inf")
    for n in nodes:
        d = np.linalg.norm(n.pos - sample)
        if d < best_d:
            best_d = d
            best = n
    return best

# =============================
# Reconstruct Path
# =============================
def reconstruct_path(node):
    path = []
    while node is not None:
        # expand back to 6D (velocities zero)
        p = node.pos
        full = np.array([p[0], p[1], p[2], 0, 0, 0])
        path.append(full)
        node = node.parent
    return np.array(path[::-1])

# =============================
# Main RRT (regular, no dynamics)
# =============================
def rrt(start, goal, obstacles, bounds, max_iter=50000, step_size=0.5, goal_thresh=1.0, padding = 0.2):

    # use real bounds
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounds

    # convert start and goal
    start_pos = np.array(start[:3])        # take first 3
    goal_pos  = np.array(goal[:3]) if len(goal) != 3 else np.array(goal)

    nodes = [Node(start_pos)]

    for it in range(max_iter):

        # goal bias
        if np.random.rand() < 0.15:
            sample = goal_pos
        else:
            sample = np.array([
                np.random.uniform(xmin, xmax),
                np.random.uniform(ymin, ymax),
                np.random.uniform(zmin, zmax)
            ])

        nn = nearest(nodes, sample)

        # steer
        direction = sample - nn.pos
        dist = np.linalg.norm(direction)
        if dist == 0:
            continue
        direction = direction / dist
        new_pos = nn.pos + direction * min(step_size, dist)

        # collision check
        if line_collides(nn.pos, new_pos, obstacles, padding):
            continue

        # add node
        new_node = Node(new_pos, parent=nn)
        nodes.append(new_node)
        
        draw_debug_point(new_pos)

        # goal check
        if np.linalg.norm(new_pos - goal_pos) < goal_thresh:
            print("Goal reached at iteration", it)
            return reconstruct_path(new_node)

    print("Failed to find a path.")
    return None

# =============================
# Example Usage
# =============================
if __name__ == "__main__":
    np.random.seed(0)

    start = np.array([0, 0, 1, 0, 0, 0])      # same input
    goal = np.array([10, 10, 2])              # same input

    obstacles = [
        AABB([3, 3, 0], [5, 5, 3]),
        AABB([7, 7, 0], [8, 8, 5])
    ]

    bounds = ((-5, 15), (-5, 15), (0, 6))      # same input

    path = rrt(start, goal, obstacles, bounds)

    if path is not None:
        print("Path (states):")
        print(path)
