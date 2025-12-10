#!/usr/bin/env python3
"""
Optimized RRT* in 2D with:
- Cached and incrementally updated KD-tree
- Reduced list comprehensions and caching of node points
- Much faster ellipse sampling (true Informed RRT*)
- Decreasing step size and adaptive goal sampling
- Node pruning using KD-tree clustering
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import math
import random

# -------------------------------------------------------------
# Utility functions
# -------------------------------------------------------------
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def steer(from_node, to_point, step):
    fx, fy = from_node
    tx, ty = to_point
    dx, dy = tx - fx, ty - fy
    d = math.hypot(dx, dy)
    if d <= step:
        return to_point
    s = step / d
    return (fx + s * dx, fy + s * dy)


def inside_obstacle(point, obstacles):
    x, y = point
    for obs in obstacles:
        t = obs["type"]
        if t == "circle":
            if dist(point, (obs["x"], obs["y"])) <= obs["r"]:
                return True
        elif t == "rect":
            if obs["x"] <= x <= obs["x"] + obs["w"] and obs["y"] <= y <= obs["y"] + obs["h"]:
                return True
    return False


def collision_free(p1, p2, obstacles, resolution=10):
    x1, y1 = p1
    x2, y2 = p2
    for t in np.linspace(0, 1, resolution):
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        if inside_obstacle((x, y), obstacles):
            return False
    return True

# -------------------------------------------------------------
# RRT* class
# -------------------------------------------------------------
class Node:
    __slots__ = ("point", "parent", "cost")
    def __init__(self, point):
        self.point = point
        self.parent = None
        self.cost = float("inf")


class DynamicKDTree:
    """Incremental KD-tree that updates only when needed."""
    def __init__(self):
        self.points = []
        self.tree = None
        self.dirty = True

    def add(self, point):
        self.points.append(point)
        self.dirty = True

    def rebuild(self):
        self.tree = KDTree(self.points) if self.points else None
        self.dirty = False

    def ensure(self):
        if self.dirty:
            self.rebuild()

    def query(self, point):
        self.ensure()
        return self.tree.query(point)

    def query_k(self, point, k):
        self.ensure()
        return self.tree.query(point, k)

    def query_radius(self, point, r):
        self.ensure()
        return self.tree.query_ball_point(point, r)


class RRTStar:
    def __init__(self, start, goal, bounds, obstacles=None,
                 step_size=5.0,
                 min_step_size=0.5,
                 goal_sample_rate=0.1,
                 max_iter=3000,
                 prune_radius=1.0,
                 k_nearest=20):

        self.start = Node(start)
        self.start.cost = 0.0
        self.goal = Node(goal)
        self.bounds = bounds
        self.obstacles = obstacles or []

        self.initial_step = step_size
        self.min_step_size = min_step_size
        self.goal_sample_rate = goal_sample_rate
        self.goal_found = False
        self.c_best = float("inf")

        self.max_iter = max_iter
        self.prune_radius = prune_radius
        self.k_nearest = k_nearest

        self.nodes = [self.start]

        # Incremental KD-tree
        self.kdtree = DynamicKDTree()
        self.kdtree.add(start)

        # Precompute ellipse rotation
        s = np.array(start)
        g = np.array(goal)
        direction = (g - s) / np.linalg.norm(g - s)
        angle = math.atan2(direction[1], direction[0])
        self.R_ellipse = np.array([[math.cos(angle), -math.sin(angle)],
                                   [math.sin(angle),  math.cos(angle)]])
        self.focal_mid = (s + g) / 2.0
        self.c_min = np.linalg.norm(g - s)

    # ---- Optimized Informed RRT* ellipse sampling ----
    def sample_in_ellipse(self):
        if not self.goal_found:
            return None
        c_best = self.c_best
        if c_best < self.c_min:
            return None

        # Semi-major/minor axes
        a = c_best / 2
        b = math.sqrt(c_best**2 - self.c_min**2) / 2

        # Sample within unit circle
        r = math.sqrt(random.random())
        t = random.random() * 2 * math.pi
        unit = np.array([r * math.cos(t), r * math.sin(t)])

        # Map to ellipse
        point = self.R_ellipse @ np.array([a, 0]) * 0  # keep shape stable
        mapped = self.R_ellipse @ (np.array([a, b]) * unit) + self.focal_mid
        x, y = mapped

        bx1, bx2, by1, by2 = self.bounds
        if bx1 <= x <= bx2 and by1 <= y <= by2:
            return (float(x), float(y))
        return None

    def random_sample(self):
        if self.goal_found:
            p = self.sample_in_ellipse()
            if p is not None:
                return p

            # reduce goal sampling rate significantly
            if random.random() < self.goal_sample_rate * 0.1:
                return self.goal.point
        else:
            if random.random() < self.goal_sample_rate:
                return self.goal.point

        bx1, bx2, by1, by2 = self.bounds
        return (random.uniform(bx1, bx2), random.uniform(by1, by2))

    # ---- Pruning ----
    def prune_nodes(self):
        to_remove = set()
        pts = self.kdtree.points

        for i, n in enumerate(self.nodes):
            if i in to_remove:
                continue
            idxs = self.kdtree.query_radius(n.point, self.prune_radius)
            if len(idxs) > 1:
                best = min(idxs, key=lambda j: self.nodes[j].cost)
                for j in idxs:
                    if j != best:
                        to_remove.add(j)

        if to_remove:
            self.nodes = [n for i, n in enumerate(self.nodes) if i not in to_remove]
            self.kdtree.points = [n.point for n in self.nodes]
            self.kdtree.dirty = True

    # ---- Main planning ----
    def plan(self):
        for k in range(self.max_iter):

            step = max(self.min_step_size,
                        self.initial_step * (0.995 ** (len(self.nodes) * 0.01)))

            q_rand = self.random_sample()
            _, idx = self.kdtree.query(q_rand)
            q_near = self.nodes[idx]

            q_new_pt = steer(q_near.point, q_rand, step)
            if inside_obstacle(q_new_pt, self.obstacles):
                continue
            if not collision_free(q_near.point, q_new_pt, self.obstacles):
                continue

            q_new = Node(q_new_pt)

            # k nearest neighbors
            k_eff = min(self.k_nearest, len(self.nodes))
            _, idxs = self.kdtree.query_k(q_new_pt, k_eff)
            if isinstance(idxs, np.int64):
                idxs = [idxs]

            best_parent = q_near
            best_cost = q_near.cost + dist(q_near.point, q_new_pt)

            for i in idxs:
                nb = self.nodes[i]
                c = nb.cost + dist(nb.point, q_new_pt)
                if c < best_cost and collision_free(nb.point, q_new_pt, self.obstacles):
                    best_cost = c
                    best_parent = nb

            q_new.parent = best_parent
            q_new.cost = best_cost

            self.nodes.append(q_new)
            self.kdtree.add(q_new_pt)

            # rewire
            for i in idxs:
                nb = self.nodes[i]
                c = q_new.cost + dist(nb.point, q_new_pt)
                if c < nb.cost and collision_free(nb.point, q_new_pt, self.obstacles):
                    nb.parent = q_new
                    nb.cost = c

            if k % 100 == 0:
                self.prune_nodes()

            # goal connect
            if dist(q_new_pt, self.goal.point) < step:
                if collision_free(q_new_pt, self.goal.point, self.obstacles):
                    self.goal_found = True
                    self.goal.parent = q_new
                    self.goal.cost = q_new.cost + dist(q_new_pt, self.goal.point)
                    self.c_best = self.goal.cost

        return self.get_path()

    def get_path(self):
        if not self.goal_found:
            return None
        path = []
        n = self.goal
        while n is not None:
            path.append(n.point)
            n = n.parent
        return path[::-1]

# -------------------------------------------------------------
# Visualization
# -------------------------------------------------------------
def plot(rrt, path=None):
    plt.figure(figsize=(8, 8))
    for n in rrt.nodes:
        if n.parent:
            x1, y1 = n.point
            x2, y2 = n.parent.point
            plt.plot([x1, x2], [y1, y2], "k-", linewidth=0.25)

    for obs in rrt.obstacles:
        if obs["type"] == "circle": plt.gca().add_patch(plt.Circle((obs["x"], obs["y"]), obs["r"], color="gray"))
        elif obs["type"] == "rect": plt.gca().add_patch(plt.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"], color="gray"))

    if path:
        xs, ys = zip(*path)
        plt.plot(xs, ys, "r-", linewidth=2)

    plt.scatter(*rrt.start.point, c="green", s=60)
    plt.scatter(*rrt.goal.point, c="red", s=60)
    bx1, bx2, by1, by2 = rrt.bounds
    plt.xlim(bx1, bx2)
    plt.ylim(by1, by2)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

# -------------------------------------------------------------
# Example
# -------------------------------------------------------------
if __name__ == "__main__":
    random.seed(0)
    np.random.seed(0)

    obstacles = [
        {"type": "circle", "x": 40, "y": 40, "r": 10},
        {"type": "rect", "x": 60, "y": 20, "w": 15, "h": 40},
    ]

    rrt = RRTStar(start=(5, 5), goal=(95, 95),
                  bounds=(0, 100, 0, 100), obstacles=obstacles,
                  step_size=5.0, min_step_size=0.5, goal_sample_rate=0.2,
                  max_iter=5000, prune_radius=2.0, k_nearest=40)

    path = rrt.plan()
    plot(rrt, path)


# #!.venv/bin/python3
# """
# RRT* in 2D with:
# - Decreasing step size as tree grows
# - Decreasing goal sample rate after goal found
# - Node pruning using k-d tree clustering
# - Informed RRT* (elliptical sampling)
# """

# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import KDTree
# import math
# import random

# # -------------------------------------------------------------
# # Utility functions
# # -------------------------------------------------------------
# def dist(a, b):
#     return np.linalg.norm(np.array(a) - np.array(b))


# def steer(from_node, to_point, step):
#     """Return a new point in direction from_node → to_point, limited by step"""
#     v = np.array(to_point) - np.array(from_node)
#     d = np.linalg.norm(v)
#     if d <= step:
#         return tuple(to_point)
#     return tuple(from_node + v / d * step)


# def inside_obstacle(point, obstacles):
#     x, y = point
#     for obs in obstacles:
#         if obs["type"] == "circle":
#             if dist((x, y), (obs["x"], obs["y"])) <= obs["r"]:
#                 return True
#         elif obs["type"] == "rect":
#             if obs["x"] <= x <= obs["x"] + obs["w"] and obs["y"] <= y <= obs["y"] + obs["h"]:
#                 return True
#     return False


# def collision_free(p1, p2, obstacles, resolution=20):
#     for t in np.linspace(0, 1, resolution):
#         x = p1[0] + t * (p2[0] - p1[0])
#         y = p1[1] + t * (p2[1] - p1[1])
#         if inside_obstacle((x, y), obstacles):
#             return False
#     return True

# # -------------------------------------------------------------
# # RRT* class
# # -------------------------------------------------------------
# class Node:
#     def __init__(self, point):
#         self.point = tuple(point)
#         self.parent = None
#         self.cost = float("inf")


# class RRTStar:
#     def __init__(self, start, goal, bounds, obstacles=None,
#                  step_size=5.0,
#                  min_step_size=0.5,
#                  goal_sample_rate=0.1,
#                  max_iter=3000,
#                  prune_radius=1.0,
#                  k_nearest=20):

#         self.start = Node(start)
#         self.start.cost = 0.0
#         self.goal = Node(goal)
#         self.bounds = bounds
#         self.obstacles = obstacles or []

#         # dynamic parameters
#         self.initial_step = step_size
#         self.min_step_size = min_step_size
#         self.goal_sample_rate = goal_sample_rate
#         self.goal_found = False

#         self.max_iter = max_iter
#         self.prune_radius = prune_radius
#         self.k_nearest = k_nearest

#         self.nodes = [self.start]

#     # ---- Informed RRT*: ellipse sampling ----
#     def sample_in_ellipse(self, c_best):
#         start = np.array(self.start.point)
#         goal = np.array(self.goal.point)
#         c_min = dist(start, goal)
#         if c_best < c_min:
#             return None

#         # ellipse major/minor axes
#         a = c_best / 2
#         b = math.sqrt(c_best**2 - c_min**2) / 2 if c_best > c_min else 0.001

#         # sample inside unit circle
#         r = math.sqrt(random.random())
#         theta = random.random() * 2 * math.pi
#         sample = np.array([r * math.cos(theta), r * math.sin(theta)])

#         # scale to ellipse
#         L = np.diag([a, b])

#         # rotation
#         direction = (goal - start) / np.linalg.norm(goal - start)
#         angle = math.atan2(direction[1], direction[0])
#         R = np.array([[math.cos(angle), -math.sin(angle)],
#                       [math.sin(angle),  math.cos(angle)]])

#         mapped = R @ (L @ sample) + (start + goal) / 2
#         x, y = mapped

#         if self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]:
#             return (x, y)
#         return None

#     # ---- General sampling ----
#     def random_sample(self):
#         # try ellipse first
#         if self.goal_found:
#             ellipse_sample = self.sample_in_ellipse(self.goal.cost)
#             if ellipse_sample is not None:
#                 return ellipse_sample

#             # also reduce goal sample rate
#             p = self.goal_sample_rate * 0.1
#             if random.random() < p:
#                 return self.goal.point
#         else:
#             if random.random() < self.goal_sample_rate:
#                 return self.goal.point

#         return (random.uniform(self.bounds[0], self.bounds[1]),
#                 random.uniform(self.bounds[2], self.bounds[3]))

#     # ---- KD Tree nearest neighbor ----
#     def kd_tree(self):
#         return KDTree([n.point for n in self.nodes])

#     # ---- Pruning nodes ----
#     def prune_nodes(self):
#         """
#         Use KD-tree to find nodes within prune_radius of each other.
#         Keep the one with lowest cost, remove others.
#         """
#         tree = self.kd_tree()
#         to_remove = set()

#         for i, n in enumerate(self.nodes):
#             if i in to_remove:
#                 continue
#             idxs = tree.query_ball_point(n.point, self.prune_radius)
#             if len(idxs) > 1:
#                 # keep node with minimum cost
#                 costs = [(self.nodes[j].cost, j) for j in idxs]
#                 _, best = min(costs)
#                 for j in idxs:
#                     if j != best:
#                         to_remove.add(j)

#         # build new list
#         self.nodes = [n for i, n in enumerate(self.nodes) if i not in to_remove]

#     # ---- Find k nearest neighbors ----
#     def k_nearest_neighbors(self, point):
#         tree = self.kd_tree()
#         d, idxs = tree.query(point, k=min(self.k_nearest, len(self.nodes)))
#         if isinstance(idxs, np.int64):
#             idxs = [idxs]
#         return [self.nodes[i] for i in idxs]

#     # ---- Main planning ----
#     def plan(self):
#         for k in range(self.max_iter):

#             # dynamic step size
#             step = max(self.min_step_size, self.initial_step * (0.99 ** (len(self.nodes) / 50)))

#             q_rand = self.random_sample()
#             tree = self.kd_tree()
#             _, nearest_idx = tree.query(q_rand)
#             q_near = self.nodes[nearest_idx]

#             q_new_pt = steer(q_near.point, q_rand, step)
#             if inside_obstacle(q_new_pt, self.obstacles):
#                 continue
#             if not collision_free(q_near.point, q_new_pt, self.obstacles):
#                 continue

#             q_new = Node(q_new_pt)

#             # choose parent
#             neighbors = self.k_nearest_neighbors(q_new.point)
#             best_parent = q_near
#             best_cost = q_near.cost + dist(q_near.point, q_new.point)

#             for nb in neighbors:
#                 if collision_free(nb.point, q_new.point, self.obstacles):
#                     c = nb.cost + dist(nb.point, q_new.point)
#                     if c < best_cost:
#                         best_parent = nb
#                         best_cost = c

#             q_new.parent = best_parent
#             q_new.cost = best_cost
#             self.nodes.append(q_new)

#             # rewire
#             for nb in neighbors:
#                 if nb == q_new:
#                     continue
#                 c = q_new.cost + dist(nb.point, q_new.point)
#                 if c < nb.cost and collision_free(nb.point, q_new.point, self.obstacles):
#                     nb.parent = q_new
#                     nb.cost = c

#             # prune after some iterations
#             if k % 50 == 0:
#                 self.prune_nodes()

#             # goal found?
#             if dist(q_new.point, self.goal.point) < step:
#                 if collision_free(q_new.point, self.goal.point, self.obstacles):
#                     self.goal.parent = q_new
#                     self.goal.cost = q_new.cost + dist(q_new.point, self.goal.point)
#                     self.goal_found = True

#         return self.get_path()

#     def get_path(self):
#         if not self.goal_found:
#             return None
#         path = []
#         n = self.goal
#         while n is not None:
#             path.append(n.point)
#             n = n.parent
#         return path[::-1]


# # -------------------------------------------------------------
# # Visualization
# # -------------------------------------------------------------
# def plot(rrt, path=None):
#     plt.figure(figsize=(8, 8))

#     for n in rrt.nodes:
#         if n.parent:
#             plt.plot([n.point[0], n.parent.point[0]], [n.point[1], n.parent.point[1]], "k-", linewidth=0.3)

#     for obs in rrt.obstacles:
#         if obs["type"] == "circle": plt.gca().add_patch(plt.Circle((obs["x"], obs["y"]), obs["r"], color="gray"))
#         elif obs["type"] == "rect": plt.gca().add_patch(plt.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"], color="gray"))

#     if path:
#         xs, ys = zip(*path)
#         plt.plot(xs, ys, "r-", linewidth=2)

#     plt.scatter(*rrt.start.point, c="green", s=80)
#     plt.scatter(*rrt.goal.point, c="red", s=80)
#     plt.xlim(rrt.bounds[0], rrt.bounds[1])
#     plt.ylim(rrt.bounds[2], rrt.bounds[3])
#     plt.gca().set_aspect("equal", adjustable="box")
#     plt.show()

# # -------------------------------------------------------------
# # Example
# # -------------------------------------------------------------
# if __name__ == "__main__":
#     obstacles = [
#         {"type": "circle", "x": 40, "y": 40, "r": 10},
#         {"type": "rect", "x": 60, "y": 20, "w": 15, "h": 40},
#     ]

#     rrt = RRTStar(start=(5, 5), goal=(95, 95), bounds=(0, 100, 0, 100), obstacles=obstacles,
#                   step_size=5.0, min_step_size=0.5, goal_sample_rate=0.2,
#                   max_iter=5000, prune_radius=2.0, k_nearest=30)

#     path = rrt.plan()
#     plot(rrt, path)
