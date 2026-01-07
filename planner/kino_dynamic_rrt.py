import math
import random
import numpy as np

from check_point import is_point_in_obstacle

# ============================================================
# Utilities
# ============================================================

def wrap_angle(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi


def normalize_state(s):
    if len(s) == 6:
        return np.array(s, dtype=float)
    if len(s) == 3:
        return np.array([s[0], s[1], s[2], 0.0, 0.0, 0.0], dtype=float)
    raise ValueError("State must be 3D or 6D")


# ============================================================
# Node
# ============================================================

class Node:
    def __init__(self, state):
        self.state = normalize_state(state)
        self.parent = None
        self.cost = 0.0


# ============================================================
# Collision helpers (uses YOUR functions)
# ============================================================

def is_state_collision_free(state, obstacles, padding):
    return not is_point_in_obstacle(state[:3], obstacles, padding)


def is_path_collision_free(path, obstacles, padding):
    for s in path:
        if not is_state_collision_free(s, obstacles, padding):
            return False
    return True


# ============================================================
# Dubins-like kinodynamic steering (no external libs)
# ============================================================

def dubins_steer(
    from_state,
    to_state,
    turning_radius,
    step_size=0.1,
    max_length=2.5
):
    x, y, z, r, p, yaw = from_state
    gx, gy, gz, _, _, _ = to_state

    states = []
    length = 0.0

    steps = int(max_length / step_size)
    dz = (gz - z) / steps if steps > 0 else 0.0

    yaw_gain = 0.6  # critical for narrow corridors

    for _ in range(steps):
        desired_yaw = math.atan2(gy - y, gx - x)
        yaw_error = wrap_angle(desired_yaw - yaw)

        max_dyaw = yaw_gain * step_size / turning_radius
        dyaw = max(-max_dyaw, min(max_dyaw, yaw_error))
        yaw = wrap_angle(yaw + dyaw)

        x += step_size * math.cos(yaw)
        y += step_size * math.sin(yaw)
        z += dz

        length += step_size
        states.append([x, y, z, r, p, yaw])

    return np.array(states), length


def dubins_distance(a, b, turning_radius):
    a = np.asarray(a)
    b = np.asarray(b)

    d_xy = np.linalg.norm(a[:2] - b[:2])
    d_z = abs(a[2] - b[2])
    d_yaw = abs(wrap_angle(a[5] - b[5])) * turning_radius

    return d_xy + d_z + d_yaw


# ============================================================
# Kinodynamic RRT*
# ============================================================

def rrt(
    start,
    goal,
    bounds,
    obstacles,
    padding=0.15,
    turning_radius=0.35,
    max_iters=7000,
    step_size=0.1,
    max_extend_length=2.5,
    search_radius=3.0,
    goal_sample_rate=0.35
):
    start = normalize_state(start)
    goal = normalize_state(goal)

    root = Node(start)
    nodes = [root]

    goal_pos_tol = 0.25  # half hole diameter
    goal_yaw_tol = 0.6

    for _ in range(max_iters):

        # ----------------------------------------------------
        # Sampling
        # ----------------------------------------------------
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            sample = np.array([
                random.uniform(bounds[0][0], bounds[0][1]),
                random.uniform(bounds[1][0], bounds[1][1]),
                random.uniform(bounds[2][0], bounds[2][1]),
                0.0,
                0.0,
                random.uniform(-math.pi, math.pi)
            ])

        # ----------------------------------------------------
        # Nearest
        # ----------------------------------------------------
        nearest = min(
            nodes,
            key=lambda n: dubins_distance(n.state, sample, turning_radius)
        )

        # ----------------------------------------------------
        # Steer
        # ----------------------------------------------------
        path, seg_cost = dubins_steer(
            nearest.state,
            sample,
            turning_radius,
            step_size,
            max_extend_length
        )

        if len(path) == 0:
            continue
        if not is_path_collision_free(path, obstacles, padding):
            continue

        new_node = Node(path[-1])
        new_node.cost = nearest.cost + seg_cost
        new_node.parent = nearest

        # ----------------------------------------------------
        # RRT* choose best parent
        # ----------------------------------------------------
        neighbors = [
            n for n in nodes
            if dubins_distance(n.state, new_node.state, turning_radius) < search_radius
        ]

        for n in neighbors:
            p, c = dubins_steer(
                n.state,
                new_node.state,
                turning_radius,
                step_size,
                max_extend_length
            )
            if len(p) == 0:
                continue
            if is_path_collision_free(p, obstacles, padding):
                cost = n.cost + c
                if cost < new_node.cost:
                    new_node.cost = cost
                    new_node.parent = n

        nodes.append(new_node)

        # ----------------------------------------------------
        # Rewire
        # ----------------------------------------------------
        for n in neighbors:
            p, c = dubins_steer(
                new_node.state,
                n.state,
                turning_radius,
                step_size,
                max_extend_length
            )
            if len(p) == 0:
                continue
            if is_path_collision_free(p, obstacles, padding):
                cost = new_node.cost + c
                if cost < n.cost:
                    n.parent = new_node
                    n.cost = cost

        # ----------------------------------------------------
        # Goal region
        # ----------------------------------------------------
        pos_dist = np.linalg.norm(new_node.state[:3] - goal[:3])
        yaw_dist = abs(wrap_angle(new_node.state[5] - goal[5]))

        if pos_dist < goal_pos_tol and yaw_dist < goal_yaw_tol:
            goal_node = Node(goal)
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + pos_dist
            nodes.append(goal_node)

            path = []
            cur = goal_node
            while cur is not None:
                path.append(cur.state.tolist())
                cur = cur.parent
            return path[::-1]

    return None
