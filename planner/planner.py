# """
# Optimal RRT (RRT*) implementation in 2D with configurable parameters.

# Features:
# - Goal bias sampling with configurable percentage (goal_sample_rate).
# - Circular and rectangular obstacles.
# - Configurable step size, max iterations, search radius factor for rewiring.
# - Visualization using matplotlib (shows tree growth and final path).

# Usage:
# - Run as a script. Edit parameters in the `if __name__ == '__main__'` block or use programmatic API.

# Author: ChatGPT
# """

# import math
# import random
# from collections import deque

# import numpy as np
# import matplotlib.pyplot as plt


# MAX_ITER = 500
# STEP_SIZE = 0.05
# GOAL_SAMPLE_RATE = 0.10
# SEARCH_RADIUS_FACTOR = 2.0
# GOAL_RADIUS = 0.03
# RANDOM_SEED = 42


# class Node:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
#         self.parent = None
#         self.cost = 0.0  # cost from start

#     def point(self):
#         return (self.x, self.y)


# class Obstacle:
#     def collides(self, p1, p2=None):
#         """Check collision. If p2 is None, just check point collision."""
#         raise NotImplementedError


# class CircleObstacle(Obstacle):
#     def __init__(self, cx, cy, r):
#         self.cx = cx
#         self.cy = cy
#         self.r = r

#     def collides(self, p1, p2=None):
#         if p2 is None:
#             x, y = p1
#             return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.r ** 2
#         # Check segment-circle intersection
#         (x1, y1), (x2, y2) = p1, p2
#         # Project center onto segment
#         dx = x2 - x1
#         dy = y2 - y1
#         if dx == 0 and dy == 0:
#             return self.collides(p1)
#         t = ((self.cx - x1) * dx + (self.cy - y1) * dy) / (dx * dx + dy * dy)
#         t = max(0.0, min(1.0, t))
#         closest_x = x1 + t * dx
#         closest_y = y1 + t * dy
#         return (closest_x - self.cx) ** 2 + (closest_y - self.cy) ** 2 <= self.r ** 2


# class RectObstacle(Obstacle):
#     def __init__(self, xmin, ymin, xmax, ymax):
#         self.xmin = min(xmin, xmax)
#         self.xmax = max(xmin, xmax)
#         self.ymin = min(ymin, ymax)
#         self.ymax = max(ymin, ymax)

#     def collides(self, p1, p2=None):
#         if p2 is None:
#             x, y = p1
#             return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax
#         # Check segment-rectangle intersection (conservative): sample points along segment
#         (x1, y1), (x2, y2) = p1, p2
#         steps = max(8, int(math.hypot(x2 - x1, y2 - y1) / 0.01))
#         for i in range(steps + 1):
#             t = i / steps
#             x = x1 + t * (x2 - x1)
#             y = y1 + t * (y2 - y1)
#             if self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax:
#                 return True
#         return False


# class RRTStar:
#     def __init__(self,
#                  start,
#                  goal,
#                  map_bounds=(0, 0, 1, 1),
#                  obstacles=None,
#                  max_iter=500,
#                  step_size=0.05,
#                  goal_sample_rate=0.05,
#                  search_radius_factor=1.0,
#                  goal_radius=0.03,
#                  seed=None):
#         """
#         start, goal: tuples (x,y)
#         map_bounds: (xmin, ymin, xmax, ymax)
#         obstacles: list of Obstacle instances
#         max_iter: max number of samples
#         step_size: incremental extension length
#         goal_sample_rate: fraction [0,1] that samples are exactly goal
#         search_radius_factor: multiplies the recommended rewiring radius
#         goal_radius: distance threshold to consider goal reached
#         """
#         self.start = Node(*start)
#         self.goal = Node(*goal)
#         self.map_bounds = map_bounds
#         self.width = map_bounds[2] - map_bounds[0]
#         self.height = map_bounds[3] - map_bounds[1]
#         self.obstacles = obstacles or []
#         self.max_iter = max_iter
#         self.step_size = step_size
#         self.goal_sample_rate = max(0.0, min(1.0, goal_sample_rate))
#         self.search_radius_factor = search_radius_factor
#         self.goal_radius = goal_radius
#         self.nodes = [self.start]
#         if seed is not None:
#             random.seed(seed)
#             np.random.seed(seed)

#     def plan(self, draw=False):
#         best_goal_node = None
#         for it in range(self.max_iter):
#             rnd = self.sample()
#             nearest = self.get_nearest(self.nodes, rnd)
#             new_pt = self.steer((nearest.x, nearest.y), rnd, self.step_size)
#             if not new_pt:
#                 continue
#             if self.collides((nearest.x, nearest.y), new_pt):
#                 continue
#             new_node = Node(*new_pt)
#             new_node.parent = nearest
#             new_node.cost = nearest.cost + self.dist((nearest.x, nearest.y), new_pt)

#             # choose parent from nearby nodes for minimal cost
#             nlist = self.near(self.nodes, new_node)
#             new_parent = nearest
#             new_cost = new_node.cost
#             for nd in nlist:
#                 if not self.collides((nd.x, nd.y), new_pt):
#                     c = nd.cost + self.dist((nd.x, nd.y), new_pt)
#                     if c < new_cost:
#                         new_parent = nd
#                         new_cost = c
#             new_node.parent = new_parent
#             new_node.cost = new_cost
#             self.nodes.append(new_node)

#             # rewire nearby nodes
#             for nd in nlist:
#                 if nd is new_node.parent:
#                     continue
#                 if self.collides((new_node.x, new_node.y), (nd.x, nd.y)):
#                     continue
#                 new_cost = new_node.cost + self.dist((new_node.x, new_node.y), (nd.x, nd.y))
#                 if new_cost < nd.cost:
#                     nd.parent = new_node
#                     nd.cost = new_cost

#             # check if it connects to goal
#             if self.dist((new_node.x, new_node.y), (self.goal.x, self.goal.y)) <= self.goal_radius:
#                 # create a goal node attached to new_node
#                 goal_node = Node(self.goal.x, self.goal.y)
#                 if not self.collides((new_node.x, new_node.y), (goal_node.x, goal_node.y)):
#                     goal_node.parent = new_node
#                     goal_node.cost = new_node.cost + self.dist((new_node.x, new_node.y), (goal_node.x, goal_node.y))
#                     # keep best
#                     if (best_goal_node is None) or (goal_node.cost < best_goal_node.cost):
#                         best_goal_node = goal_node

#             if draw and (it % max(1, self.max_iter // 200) == 0):
#                 self.draw(show=False, best=best_goal_node)

#         path = None
#         if best_goal_node:
#             path = self.extract_path(best_goal_node)

#         return path, best_goal_node

#     def sample(self):
#         if random.random() < self.goal_sample_rate:
#             return (self.goal.x, self.goal.y)
#         xmin, ymin, xmax, ymax = self.map_bounds
#         return (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

#     def steer(self, from_pt, to_pt, max_dist):
#         (x1, y1) = from_pt
#         (x2, y2) = to_pt
#         dx = x2 - x1
#         dy = y2 - y1
#         dist = math.hypot(dx, dy)
#         if dist == 0:
#             return None
#         if dist <= max_dist:
#             return (x2, y2)
#         theta = math.atan2(dy, dx)
#         return (x1 + max_dist * math.cos(theta), y1 + max_dist * math.sin(theta))

#     def collides(self, p1, p2=None):
#         # Check if segment p1-p2 intersects any obstacle
#         # Also check endpoints inside obstacles
#         for ob in self.obstacles:
#             if ob.collides(p1, p2):
#                 return True
#         return False

#     def get_nearest(self, nodes, point):
#         best = nodes[0]
#         best_d = self.dist((best.x, best.y), point)
#         for n in nodes:
#             d = self.dist((n.x, n.y), point)
#             if d < best_d:
#                 best = n
#                 best_d = d
#         return best

#     def near(self, nodes, node):
#         # radius based on RRT* theoretical radius: r = gamma * (log(n)/n)^{1/d}
#         n = len(nodes) + 1
#         gamma = self.search_radius_factor * (2 * (1 + 1/2) ** (1/2))  # heuristic gamma
#         dim = 2
#         r = min(self.step_size * 50.0, gamma * ((math.log(n) / n) ** (1.0 / dim)))
#         # ensure minimum radius at least step_size
#         r = max(r, self.step_size * 1.5)
#         near_nodes = [nd for nd in nodes if self.dist((nd.x, nd.y), (node.x, node.y)) <= r]
#         return near_nodes

#     @staticmethod
#     def dist(a, b):
#         return math.hypot(a[0] - b[0], a[1] - b[1])

#     def extract_path(self, goal_node):
#         path = []
#         node = goal_node
#         while node is not None:
#             path.append((node.x, node.y))
#             node = node.parent
#         path.reverse()
#         return path

#     def draw(self, show=True, best=None):
#         plt.clf()
#         xmin, ymin, xmax, ymax = self.map_bounds
#         plt.xlim(xmin, xmax)
#         plt.ylim(ymin, ymax)
#         # draw obstacles
#         ax = plt.gca()
#         for ob in self.obstacles:
#             if isinstance(ob, CircleObstacle):
#                 circ = plt.Circle((ob.cx, ob.cy), ob.r, fill=True, alpha=0.6)
#                 ax.add_patch(circ)
#             elif isinstance(ob, RectObstacle):
#                 rect = plt.Rectangle((ob.xmin, ob.ymin), ob.xmax - ob.xmin, ob.ymax - ob.ymin, alpha=0.6)
#                 ax.add_patch(rect)

#         # draw tree
#         for n in self.nodes:
#             if n.parent is not None:
#                 plt.scatter(n.x, n.y)
#                 # plt.plot([n.x, n.parent.x], [n.y, n.parent.y])

#         # draw start/goal
#         plt.scatter(self.start.x, self.start.y, marker='o', c='green', s=50, label='start')
#         plt.scatter(self.goal.x, self.goal.y, marker='x', c='red', s=50, label='goal')

#         # draw best path if exists
#         if best is not None:
#             path = self.extract_path(best)
#             xs = [p[0] for p in path]
#             ys = [p[1] for p in path]
#             plt.plot(xs, ys, linewidth=3, label='best path')

#         plt.gca().set_aspect('equal', adjustable='box')
#         plt.legend()
#         plt.pause(0.001)
#         if show:
#             plt.show()


# if __name__ == '__main__':
#     # Example usage and parameter configuration
#     start = (0.05, 0.05)
#     goal = (0.95, 0.95)
#     map_bounds = (0.0, 0.0, 1.0, 1.0)

#     # Define obstacles (mix of circles and rectangles)
#     obstacles = [
#         CircleObstacle(0.4, 0.4, 0.12),
#         CircleObstacle(0.7, 0.6, 0.10),
#         RectObstacle(0.2, 0.7, 0.45, 0.85),
#         RectObstacle(0.6, 0.15, 0.9, 0.3),
#     ]

#     planner = RRTStar(start=start,
#                       goal=goal,
#                       map_bounds=map_bounds,
#                       obstacles=obstacles,
#                       max_iter=MAX_ITER,
#                       step_size=STEP_SIZE,
#                       goal_sample_rate=GOAL_SAMPLE_RATE,
#                       search_radius_factor=SEARCH_RADIUS_FACTOR,
#                       goal_radius=GOAL_RADIUS,
#                       seed=RANDOM_SEED)

#     plt.ion()
#     path, goal_node = planner.plan(draw=True)
#     planner.draw(show=True, best=goal_node)

#     if path:
#         print('Found path with %d waypoints, cost=%.3f' % (len(path), goal_node.cost))
#     else:
#         print('No path found (increase iterations/step size or adjust obstacles).')

#     # Keep plot open
#     plt.ioff()
#     plt.show()

"""
Optimal RRT (RRT*) implementation in 2D with configurable parameters.

Features:
- Adaptive step size that decays as the tree grows.
- Adaptive goal sampling: goal_sample_rate reduced after first goal found.
- Pruning of nearby nodes (cost-based dominance).
- Circular and rectangular obstacles.
- Visualization using matplotlib (shows tree growth and final path).

Usage:
- Run as a script. Edit parameters in the `if __name__ == '__main__'` block or use programmatic API.
"""

import math
import random
from collections import deque

import numpy as np
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0  # cost from start

    def point(self):
        return (self.x, self.y)


class Obstacle:
    def collides(self, p1, p2=None):
        """Check collision. If p2 is None, just check point collision."""
        raise NotImplementedError


class CircleObstacle(Obstacle):
    def __init__(self, cx, cy, r):
        self.cx = cx
        self.cy = cy
        self.r = r

    def collides(self, p1, p2=None):
        if p2 is None:
            x, y = p1
            return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.r ** 2
        # Check segment-circle intersection
        (x1, y1), (x2, y2) = p1, p2
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return self.collides(p1)
        t = ((self.cx - x1) * dx + (self.cy - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return (closest_x - self.cx) ** 2 + (closest_y - self.cy) ** 2 <= self.r ** 2


class RectObstacle(Obstacle):
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin = min(xmin, xmax)
        self.xmax = max(xmin, xmax)
        self.ymin = min(ymin, ymax)
        self.ymax = max(ymin, ymax)

    def collides(self, p1, p2=None):
        if p2 is None:
            x, y = p1
            return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax
        # Check segment-rectangle intersection (conservative): sample points along segment
        (x1, y1), (x2, y2) = p1, p2
        steps = max(8, int(math.hypot(x2 - x1, y2 - y1) / 0.01))
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax:
                return True
        return False


class RRTStar:
    def __init__(self,
                 start,
                 goal,
                 map_bounds=(0, 0, 1, 1),
                 obstacles=None,
                 max_iter=500,
                 step_size=0.05,
                 goal_sample_rate=0.05,
                 search_radius_factor=1.0,
                 goal_radius=0.03,
                 prune_radius=0.01,
                 step_decay=0.0005,
                 goal_sample_rate_decay=0.5,
                 seed=None):
        """
        start, goal: tuples (x,y)
        map_bounds: (xmin, ymin, xmax, ymax)
        obstacles: list of Obstacle instances
        max_iter: max number of samples
        step_size: base incremental extension length
        goal_sample_rate: fraction [0,1] that samples are exactly goal
        search_radius_factor: multiplies the recommended rewiring radius
        goal_radius: distance threshold to consider goal reached
        prune_radius: nodes closer than this may be pruned
        step_decay: how strongly step size shrinks with number of nodes
        goal_sample_rate_decay: multiplicative factor applied to goal_sample_rate after first goal found
        """
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_bounds = map_bounds
        self.width = map_bounds[2] - map_bounds[0]
        self.height = map_bounds[3] - map_bounds[1]
        self.obstacles = obstacles or []
        self.max_iter = max_iter
        self.base_step_size = step_size
        self.base_goal_sample_rate = max(0.0, min(1.0, goal_sample_rate))
        self.goal_sample_rate = self.base_goal_sample_rate
        self.search_radius_factor = search_radius_factor
        self.goal_radius = goal_radius
        self.nodes = [self.start]
        self.found_goal_once = False
        self.prune_radius = prune_radius
        self.step_decay = step_decay
        self.goal_sample_rate_decay = goal_sample_rate_decay
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

    def plan(self, draw=False):
        best_goal_node = None
        for it in range(self.max_iter):
            # update adaptive parameters (step size implicitly used in steer)
            rnd = self.sample()
            nearest = self.get_nearest(self.nodes, rnd)
            new_pt = self.steer((nearest.x, nearest.y), rnd)
            if not new_pt:
                continue
            if self.collides((nearest.x, nearest.y), new_pt):
                continue
            new_node = Node(*new_pt)
            new_node.parent = nearest
            new_node.cost = nearest.cost + self.dist((nearest.x, nearest.y), new_pt)

            # prune nodes near new_node (cost-based)
            if self.prune_radius > 0:
                self.prune_nearby(new_node)

            # choose parent from nearby nodes for minimal cost
            nlist = self.near(self.nodes, new_node)
            new_parent = nearest
            new_cost = new_node.cost
            for nd in nlist:
                if not self.collides((nd.x, nd.y), new_pt):
                    c = nd.cost + self.dist((nd.x, nd.y), new_pt)
                    if c < new_cost:
                        new_parent = nd
                        new_cost = c
            new_node.parent = new_parent
            new_node.cost = new_cost
            self.nodes.append(new_node)

            # rewire nearby nodes
            for nd in nlist:
                if nd is new_node.parent:
                    continue
                if self.collides((new_node.x, new_node.y), (nd.x, nd.y)):
                    continue
                new_cost = new_node.cost + self.dist((new_node.x, new_node.y), (nd.x, nd.y))
                if new_cost < nd.cost:
                    nd.parent = new_node
                    nd.cost = new_cost

            # check if it connects to goal
            if self.dist((new_node.x, new_node.y), (self.goal.x, self.goal.y)) <= self.goal_radius:
                # create a goal node attached to new_node
                goal_node = Node(self.goal.x, self.goal.y)
                if not self.collides((new_node.x, new_node.y), (goal_node.x, goal_node.y)):
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + self.dist((new_node.x, new_node.y), (goal_node.x, goal_node.y))
                    # keep best
                    if (best_goal_node is None) or (goal_node.cost < best_goal_node.cost):
                        best_goal_node = goal_node
                    self.found_goal_once = True

            if draw and (it % max(1, self.max_iter // 200) == 0):
                self.draw(show=False, best=best_goal_node)

        path = None
        if best_goal_node:
            path = self.extract_path(best_goal_node)

        return path, best_goal_node

    def sample(self):
        # reduce goal sampling rate once goal seen
        if self.found_goal_once:
            eff_goal_sample_rate = self.base_goal_sample_rate * self.goal_sample_rate_decay
        else:
            eff_goal_sample_rate = self.base_goal_sample_rate

        if random.random() < eff_goal_sample_rate:
            return (self.goal.x, self.goal.y)
        xmin, ymin, xmax, ymax = self.map_bounds
        return (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

    def steer(self, from_pt, to_pt):
        (x1, y1) = from_pt
        (x2, y2) = to_pt
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        if dist == 0:
            return None
        # adaptive step: base_step_size / (1 + step_decay * n_nodes)
        adaptive_step = self.base_step_size / (1.0 + self.step_decay * len(self.nodes))
        if dist <= adaptive_step:
            return (x2, y2)
        theta = math.atan2(dy, dx)
        return (x1 + adaptive_step * math.cos(theta), y1 + adaptive_step * math.sin(theta))

    def collides(self, p1, p2=None):
        # Check if segment p1-p2 intersects any obstacle
        # Also check endpoints inside obstacles
        for ob in self.obstacles:
            if ob.collides(p1, p2):
                return True
        return False

    def get_nearest(self, nodes, point):
        best = nodes[0]
        best_d = self.dist((best.x, best.y), point)
        for n in nodes:
            d = self.dist((n.x, n.y), point)
            if d < best_d:
                best = n
                best_d = d
        return best

    def near(self, nodes, node):
        # radius based on RRT* theoretical radius: r = gamma * (log(n)/n)^{1/d}
        n = len(nodes) + 1
        gamma = self.search_radius_factor * (2 * (1 + 1/2) ** (1/2))  # heuristic gamma
        dim = 2
        # ensure n>1 for log
        if n <= 1:
            r = self.base_step_size * 2.0
        else:
            r = min(self.base_step_size * 50.0, gamma * ((math.log(n) / n) ** (1.0 / dim)))
        # ensure minimum radius at least step_size
        r = max(r, self.base_step_size * 1.5)
        near_nodes = [nd for nd in nodes if self.dist((nd.x, nd.y), (node.x, node.y)) <= r]
        return near_nodes

    def prune_nearby(self, new_node):
        # Remove nearby nodes that are strictly worse (higher cost) than new_node.
        # If a nearby node has lower cost than new_node, we keep the existing node and do not remove new_node.
        to_remove = []
        for nd in list(self.nodes):
            if nd is new_node.parent:
                continue
            d = self.dist((nd.x, nd.y), (new_node.x, new_node.y))
            if d < self.prune_radius:
                # If nd has higher cost, mark it for removal
                if nd.cost > new_node.cost:
                    to_remove.append(nd)
                else:
                    # keep better node; abandon pruning (do not remove new_node)
                    return
        for nd in to_remove:
            try:
                self.nodes.remove(nd)
            except ValueError:
                pass

    @staticmethod
    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def extract_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    def draw(self, show=True, best=None):
        plt.clf()
        xmin, ymin, xmax, ymax = self.map_bounds
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        # draw obstacles
        ax = plt.gca()
        for ob in self.obstacles:
            if isinstance(ob, CircleObstacle):
                circ = plt.Circle((ob.cx, ob.cy), ob.r, fill=True, alpha=0.6)
                ax.add_patch(circ)
            elif isinstance(ob, RectObstacle):
                rect = plt.Rectangle((ob.xmin, ob.ymin), ob.xmax - ob.xmin, ob.ymax - ob.ymin, alpha=0.6)
                ax.add_patch(rect)

        # draw tree
        for n in self.nodes:
            if n.parent is not None:
                plt.scatter(n.x, n.y)
                # plt.plot([n.x, n.parent.x], [n.y, n.parent.y])

        # draw start/goal
        plt.scatter(self.start.x, self.start.y, marker='o', s=50, label='start', c="darkgreen")
        plt.scatter(self.goal.x, self.goal.y, marker='x', s=50, label='goal', c="red")

        # draw best path if exists
        if best is not None:
            path = self.extract_path(best)
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            plt.plot(xs, ys, linewidth=3, label='best path')

        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend()
        plt.pause(0.001)
        if show:
            plt.show()


if __name__ == '__main__':
    # Example usage and parameter configuration
    start = (0.05, 0.05)
    goal = (0.95, 0.95)
    map_bounds = (0.0, 0.0, 1.0, 1.0)

    # Define obstacles (mix of circles and rectangles)
    obstacles = [
        CircleObstacle(0.4, 0.4, 0.12),
        CircleObstacle(0.7, 0.6, 0.10),
        RectObstacle(0.2, 0.7, 0.45, 0.85),
        RectObstacle(0.6, 0.15, 0.9, 0.3),
    ]

    # Configurable parameters
    params = {
        'max_iter': 500,
        'step_size': 0.05,
        'goal_sample_rate': 0.10,  # 10% of samples are the goal initially
        'search_radius_factor': 2.0,
        'goal_radius': 0.03,
        'prune_radius': 0.02,
        'step_decay': 0.0008,
        'goal_sample_rate_decay': 0.25,  # reduce goal sampling to 25% of base after first goal
        'seed': 42,
    }

    planner = RRTStar(start=start,
                      goal=goal,
                      map_bounds=map_bounds,
                      obstacles=obstacles,
                      max_iter=params['max_iter'],
                      step_size=params['step_size'],
                      goal_sample_rate=params['goal_sample_rate'],
                      search_radius_factor=params['search_radius_factor'],
                      goal_radius=params['goal_radius'],
                      prune_radius=params['prune_radius'],
                      step_decay=params['step_decay'],
                      goal_sample_rate_decay=params['goal_sample_rate_decay'],
                      seed=params['seed'])

    plt.ion()
    path, goal_node = planner.plan(draw=True)
    planner.draw(show=True, best=goal_node)

    if path:
        print('Found path with %d waypoints, cost=%.3f' % (len(path), goal_node.cost))
    else:
        print('No path found (increase iterations/step size or adjust obstacles).')

    # Keep plot open
    plt.ioff()
    plt.show()
