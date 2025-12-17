#!/usr/bin/env python3
"""
Optimized RRT* in 3D with:
- Cached and incrementally updated KD-tree
- Decreasing step size and adaptive goal sampling
- Node pruning using KD-tree clustering
- Elliptical informed sampling in 3D
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import math
import random

# -------------------------------------------------------------
# Utility functions
# -------------------------------------------------------------
def dist(a, b):
    """Euclidean distance for 2D or 3D points."""
    a = np.array(a)
    b = np.array(b)
    return float(np.linalg.norm(a - b))


def steer(from_node, to_point, step):
    """Steer from from_node towards to_point, limited by step size."""
    from_arr = np.array(from_node)
    to_arr = np.array(to_point)
    direction = to_arr - from_arr
    d = np.linalg.norm(direction)
    if d <= step:
        return tuple(to_point)
    normalized = direction / d
    return tuple(from_arr + step * normalized)


def inside_obstacle(point, obstacles):
    """Check if a point is inside any obstacle (2D or 3D)."""
    for obs in obstacles:
        t = obs["type"]
        if t == "sphere":
            center = tuple(obs[k] for k in ["x", "y", "z"])
            if dist(point, center) <= obs["r"]:
                return True
        elif t == "circle":
            # 2D obstacle in 3D space (at some z-level)
            center_2d = (obs["x"], obs["y"])
            point_2d = point[:2] if len(point) > 2 else point
            if dist(point_2d, center_2d) <= obs["r"]:
                return True
        elif t == "box":
            # 3D axis-aligned box
            if (obs["x"] <= point[0] <= obs["x"] + obs["w"] and
                obs["y"] <= point[1] <= obs["y"] + obs["h"] and
                obs["z"] <= point[2] <= obs["z"] + obs["d"]):
                return True
        elif t == "rect":
            # 2D rectangle in 3D space
            if (obs["x"] <= point[0] <= obs["x"] + obs["w"] and
                obs["y"] <= point[1] <= obs["y"] + obs["h"]):
                return True
    return False


def collision_free(p1, p2, obstacles, resolution=10):
    """Check if path between p1 and p2 is collision-free."""
    p1_arr = np.array(p1)
    p2_arr = np.array(p2)
    for t in np.linspace(0, 1, resolution):
        point = p1_arr + t * (p2_arr - p1_arr)
        if inside_obstacle(tuple(point), obstacles):
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

    def add(self, point):
        self.points.append(point)

    def query(self, point):
        self.tree = KDTree(self.points) if self.points else None
        return self.tree.query(point)

    def query_k(self, point, k):
        self.tree = KDTree(self.points) if self.points else None
        return self.tree.query(point, k)

    def query_radius(self, point, r):
        self.tree = KDTree(self.points) if self.points else None
        return self.tree.query_ball_point(point, r)


class RRTStar:
    def __init__(self, start, goal, bounds, obstacles=None,
                 step_size=5.0,
                 min_step_size=0.5,
                 goal_sample_rate=0.1,
                 max_iter=1000,
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
        self.goal_radius = 1
        self.goal_found = False
        self.c_best = float("inf")

        self.max_iter = max_iter
        self.prune_radius = prune_radius
        self.k_nearest = k_nearest

        self.nodes = [self.start]

        # Incremental KD-tree
        self.kdtree = DynamicKDTree()
        self.kdtree.add(start)

        # Precompute ellipse rotation in 3D
        self.start_arr = np.array(start)
        self.goal_arr = np.array(goal)
        self.c_min = np.linalg.norm(self.goal_arr - self.start_arr)
        self.focal_mid = (self.start_arr + self.goal_arr) / 2.0

    # ---- Optimized Informed RRT* ellipse sampling in 3D ----
    def sample_in_ellipse(self):
        if not self.goal_found:
            return None
        c_best = self.c_best
        if c_best < self.c_min:
            return None

        # Semi-major/minor axes in 3D
        a = c_best / 2.0
        b = math.sqrt(max(0, c_best**2 - self.c_min**2)) / 2.0
        
        # Sample uniformly in a ball
        r = (random.random()) ** (1/3)  # Uniform in 3D ball
        phi = random.random() * 2 * math.pi
        cos_theta = random.random() * 2 - 1
        sin_theta = math.sqrt(max(0, 1 - cos_theta**2))
        
        # Map to ellipsoid: use first 2 dims as ellipse, 3rd dim as smaller
        x = a * r * math.cos(phi) * sin_theta
        y = b * r * math.sin(phi) * sin_theta
        z = b * r * cos_theta
        unit = np.array([x, y, z])
        
        # Direction from start to goal
        direction = self.goal_arr - self.start_arr
        norm_dir = direction / self.c_min
        
        # Build orthonormal basis
        if abs(norm_dir[2]) < 0.99:
            v1 = np.array([-norm_dir[1], norm_dir[0], 0])
            v1 = v1 / np.linalg.norm(v1)
        else:
            v1 = np.array([1, 0, 0])
        v2 = np.cross(norm_dir, v1)
        v2 = v2 / np.linalg.norm(v2)
        
        # Rotate unit sample to world space
        mapped = norm_dir * unit[0] + v1 * unit[1] + v2 * unit[2] + self.focal_mid
        
        bx1, bx2, by1, by2, bz1, bz2 = self.bounds
        if (bx1 <= mapped[0] <= bx2 and by1 <= mapped[1] <= by2 and 
            bz1 <= mapped[2] <= bz2):
            return tuple(mapped)
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

        bx1, bx2, by1, by2, bz1, bz2 = self.bounds
        return (random.uniform(bx1, bx2), random.uniform(by1, by2), random.uniform(bz1, bz2))

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
            if dist(q_new_pt, self.goal.point) < self.goal_radius:
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
    """Plot RRT tree and path in 3D or 2D depending on configuration."""
    fig = plt.figure(figsize=(12, 10))
    
    # Check if 3D or 2D
    is_3d = len(rrt.start.point) == 3
    
    if is_3d:
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot tree edges
        for n in rrt.nodes:
            if n.parent:
                x1, y1, z1 = n.point
                x2, y2, z2 = n.parent.point
                ax.plot([x1, x2], [y1, y2], [z1, z2], "k-", linewidth=0.25, alpha=0.6)
        
        # Plot obstacles
        for obs in rrt.obstacles:
            if obs["type"] == "sphere":
                u = np.linspace(0, 2 * np.pi, 30)
                v = np.linspace(0, np.pi, 20)
                x = obs["r"] * np.outer(np.cos(u), np.sin(v)) + obs["x"]
                y = obs["r"] * np.outer(np.sin(u), np.sin(v)) + obs["y"]
                z = obs["r"] * np.outer(np.ones(np.size(u)), np.cos(v)) + obs["z"]
                ax.plot_surface(x, y, z, color="gray", alpha=0.3)
            elif obs["type"] == "box":
                # Draw box wireframe
                x_min, x_max = obs["x"], obs["x"] + obs["w"]
                y_min, y_max = obs["y"], obs["y"] + obs["h"]
                z_min, z_max = obs["z"], obs["z"] + obs["d"]
                vertices = [
                    [x_min, y_min, z_min], [x_max, y_min, z_min],
                    [x_max, y_max, z_min], [x_min, y_max, z_min],
                    [x_min, y_min, z_max], [x_max, y_min, z_max],
                    [x_max, y_max, z_max], [x_min, y_max, z_max]
                ]
                vertices = np.array(vertices)
                # Draw edges
                edges = [
                    [0, 1], [1, 2], [2, 3], [3, 0],  # bottom
                    [4, 5], [5, 6], [6, 7], [7, 4],  # top
                    [0, 4], [1, 5], [2, 6], [3, 7]   # vertical
                ]
                for edge in edges:
                    pts = vertices[edge]
                    ax.plot3D(*pts.T, "gray", linewidth=2, alpha=0.6)
        
        # Plot path
        if path:
            xs, ys, zs = zip(*path)
            ax.plot(xs, ys, zs, "r-", linewidth=3, label="Path")
        
        # Plot start and goal
        ax.scatter(*rrt.start.point, c="green", s=100, label="Start", marker="o")
        ax.scatter(*rrt.goal.point, c="red", s=100, label="Goal", marker="*")
        
        bx1, bx2, by1, by2, bz1, bz2 = rrt.bounds
        ax.set_xlim(bx1, bx2)
        ax.set_ylim(by1, by2)
        ax.set_zlim(bz1, bz2)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()
    else:
        # 2D visualization
        ax = fig.add_subplot(111)
        
        for n in rrt.nodes:
            if n.parent:
                x1, y1 = n.point
                x2, y2 = n.parent.point
                ax.plot([x1, x2], [y1, y2], "k-", linewidth=0.25)
        
        for obs in rrt.obstacles:
            if obs["type"] == "circle":
                ax.add_patch(plt.Circle((obs["x"], obs["y"]), obs["r"], color="gray", alpha=0.5))
            elif obs["type"] == "rect":
                ax.add_patch(plt.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"], color="gray", alpha=0.5))
        
        if path:
            xs, ys = zip(*path)
            ax.plot(xs, ys, "r-", linewidth=2)
        
        ax.scatter(*rrt.start.point, c="green", s=100, label="Start")
        ax.scatter(*rrt.goal.point, c="red", s=100, label="Goal")
        bx1, bx2, by1, by2 = rrt.bounds[:4]
        ax.set_xlim(bx1, bx2)
        ax.set_ylim(by1, by2)
        ax.set_aspect("equal", adjustable="box")
        ax.legend()
    
    plt.show()

# -------------------------------------------------------------
# Example
# -------------------------------------------------------------
if __name__ == "__main__":
    random.seed(0)
    np.random.seed(0)

    # Example 3D planning
    obstacles_3d = [
        {"type": "sphere", "x": 40, "y": 40, "z": 40, "r": 10},
        {"type": "box", "x": 60, "y": 20, "z": 30, "w": 15, "h": 40, "d": 20},
        {"type": "box", "x": 50, "y": 70, "z": 50, "w": 20, "h": 10, "d": 15},
        {"type": "sphere", "x": 90, "y": 85, "z": 60, "r": 8}
    ]

    rrt = RRTStar(start=(5, 5, 5), goal=(95, 95, 90),
                  bounds=(0, 100, 0, 100, 0, 100), obstacles=obstacles_3d,
                  step_size=5.0, min_step_size=0.5, goal_sample_rate=0.2,
                  max_iter=5000, prune_radius=2.0, k_nearest=40)

    path = rrt.plan()
    plot(rrt, path)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        print(f"Path cost: {rrt.goal.cost:.2f}")
    else:
        print("No path found")


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
