import random
import math
import numpy as np

from check_point import draw_debug_point, is_point_in_obstacle

class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.parent = None
        self.cost = 0.0


def distance(a, b):
    return np.linalg.norm(a - b)


def steer(from_node, to_pos, step_size):
    direction = to_pos - from_node.pos
    dist = np.linalg.norm(direction)
    if dist <= step_size:
        return to_pos
    return from_node.pos + direction / dist * step_size


def is_collision_free(point, obstacles, padding):
    return not is_point_in_obstacle(point, obstacles, padding)


def is_segment_collision_free(p1, p2, obstacles, padding, resolution=0.05):
    dist = np.linalg.norm(p2 - p1)
    steps = max(1, int(dist / resolution))
    for i in range(steps + 1):
        p = p1 + (p2 - p1) * i / steps
        if not is_collision_free(p, obstacles, padding):
            return False
    return True


def rrt(start,
        goal,
        bounds,
        obstacles,
        padding=0.15,
        max_iters=5000,
        step_size=0.3,
        goal_sample_rate=0.1,
        search_radius=1.0):

    start_pos = np.array(start[:3])
    goal_pos = np.array(goal)

    root = Node(start_pos)
    nodes = [root]

    for _ in range(max_iters):

        # --- Sample ---
        if random.random() < goal_sample_rate:
            sample = goal_pos
        else:
            sample = np.array([
                random.uniform(bounds[0][0], bounds[0][1]),
                random.uniform(bounds[1][0], bounds[1][1]),
                random.uniform(bounds[2][0], bounds[2][1])
            ])

        # --- Nearest ---
        nearest = min(nodes, key=lambda n: distance(n.pos, sample))

        # --- Steer ---
        new_pos = steer(nearest, sample, step_size)

        if not is_collision_free(new_pos, obstacles, padding):
            continue

        if not is_segment_collision_free(nearest.pos, new_pos, obstacles, padding):
            continue

        new_node = Node(new_pos)

        # --- Choose best parent (RRT*) ---
        neighbors = [
            n for n in nodes
            if distance(n.pos, new_node.pos) <= search_radius
        ]

        best_parent = nearest
        best_cost = nearest.cost + distance(nearest.pos, new_node.pos)

        for n in neighbors:
            if is_segment_collision_free(n.pos, new_node.pos, obstacles, padding):
                cost = n.cost + distance(n.pos, new_node.pos)
                if cost < best_cost:
                    best_parent = n
                    best_cost = cost

        new_node.parent = best_parent
        new_node.cost = best_cost
        nodes.append(new_node)

        # --- Rewire ---
        for n in neighbors:
            if n == best_parent:
                continue
            if is_segment_collision_free(new_node.pos, n.pos, obstacles, padding):
                new_cost = new_node.cost + distance(new_node.pos, n.pos)
                if new_cost < n.cost:
                    n.parent = new_node
                    n.cost = new_cost

        # --- Goal check ---
        if distance(new_node.pos, goal_pos) < step_size:
            if is_segment_collision_free(new_node.pos, goal_pos, obstacles, padding):
                goal_node = Node(goal_pos)
                goal_node.parent = new_node
                nodes.append(goal_node)

                # --- Extract path ---
                path = []
                current = goal_node
                while current is not None:
                    path.append(current.pos.tolist())
                    current = current.parent
                return np.array(path[::-1])
            
        draw_debug_point(new_pos)

    return None