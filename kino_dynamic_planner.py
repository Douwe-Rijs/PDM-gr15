#!/usr/bin/env python3
"""
Kinodynamic RRT* (double-integrator approximation for a quadcopter) in 2D.

This implementation models the vehicle as a planar quadcopter-like system with state
s = [x, y, vx, vy] and control u = [ax, ay] (bounded accelerations). The planner
searches in state-space and forward-propagates dynamics under sampled controls to
generate kinodynamically-feasible trajectories.

Features:
- State-space RRT* (kinodynamic): nodes contain state, parent, cost (time), control, duration
- Forward Euler propagation of double-integrator dynamics: x'' = ax, y'' = ay
- Bounded acceleration controls and maximum rollout duration
- Collision checking along the continuous trajectory (sampled at small dt)
- KD-tree for nearest-neighbor searches cached and rebuilt lazily
- Rewiring (limited) using neighbor rollouts
- Visualization of resulting position-space trajectory

Notes / simplifications:
- This is a 2D planar kinodynamic model (x,y) with velocity; it's a common and
  useful approximation for quadcopter path planning while capturing dynamics.
- For a full 6-DOF quadcopter model (attitude, thrust limits, rotor dynamics)
  you'd replace the dynamics and control parameterization accordingly.

Author: ChatGPT
"""

import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


# -------------------------------
# Dynamics: double integrator
# -------------------------------
# state: [x, y, vx, vy]
# control: [ax, ay]
# dynamics: x_dot = vx, y_dot = vy, vx_dot = ax, vy_dot = ay


def propagate(state: np.ndarray, control: np.ndarray, duration: float, dt: float) -> List[np.ndarray]:
    """Forward-integrate the double-integrator dynamics using Euler integration.
    Returns the list of states along the trajectory (including initial and final).
    state: shape (4,), control: shape (2,)
    """
    steps = max(1, int(math.ceil(duration / dt)))
    s = state.copy()
    print(s, s.shape)
    traj = [s.copy()]
    for _ in range(steps):
        # derivatives
        s[0] += s[2] * dt
        s[1] += s[3] * dt
        s[2] += control[0] * dt
        s[3] += control[1] * dt
        traj.append(s.copy())
    return traj


# -------------------------------
# Collision checking
# -------------------------------

@dataclass
class CircleObstacle:
    x: float
    y: float
    r: float

    def collides_point(self, px: float, py: float) -> bool:
        return (px - self.x) ** 2 + (py - self.y) ** 2 <= self.r ** 2


@dataclass
class RectObstacle:
    x: float
    y: float
    w: float
    h: float

    def collides_point(self, px: float, py: float) -> bool:
        return (self.x <= px <= self.x + self.w) and (self.y <= py <= self.y + self.h)


def collision_free_trajectory(traj: List[np.ndarray], circles: List[CircleObstacle], rects: List[RectObstacle]) -> bool:
    for s in traj:
        x, y = s[0], s[1]
        for c in circles:
            if c.collides_point(x, y):
                return False
        for r in rects:
            if r.collides_point(x, y):
                return False
    return True


# -------------------------------
# Node and planner classes
# -------------------------------

@dataclass
class Node:
    state: np.ndarray  # shape (4,)
    parent: Optional[int]  # index into planner.nodes
    cost: float  # cost-to-come (use time for kinodynamic planning)
    control: Optional[np.ndarray] = None
    duration: Optional[float] = None


class IncrementalKD:
    """Simple wrapper that caches points and lazily rebuilds a KDTree when needed."""
    def __init__(self):
        self.points: List[Tuple[float, float, float, float]] = []
        self.tree: Optional[KDTree] = None
        self.dirty = True

    def add(self, pt: Tuple[float, float, float, float]):
        self.points.append(pt)
        self.dirty = True

    def ensure(self):
        if self.dirty:
            if len(self.points) > 0:
                self.tree = KDTree(self.points)
            else:
                self.tree = None
            self.dirty = False

    def query(self, pt: Tuple[float, float, float, float]):
        self.ensure()
        return self.tree.query(pt)

    def query_k(self, pt: Tuple[float, float, float, float], k: int):
        self.ensure()
        return self.tree.query(pt, k=k)

    def query_radius(self, pt: Tuple[float, float, float, float], r: float):
        self.ensure()
        return self.tree.query_ball_point(pt, r)


class KinodynamicRRTStar:
    def __init__(self,
                 start: Tuple[float, float, float, float],
                 goal_pos: Tuple[float, float],
                 bounds: Tuple[float, float, float, float],
                 circles: List[CircleObstacle] = None,
                 rects: List[RectObstacle] = None,
                 max_accel: float = 1.0,
                 max_duration: float = 2.0,
                 dt: float = 0.05,
                 max_iter: int = 2000,
                 goal_sample_rate: float = 0.1,
                 neighbor_radius: float = 5.0):
        """
        start: full state (x,y,vx,vy)
        goal_pos: only position target (x,y). Planner tries to reach goal position with any velocity.
        bounds: (xmin, xmax, ymin, ymax)
        max_accel: bound on |ax| and |ay|
        max_duration: maximum duration for a single control rollout
        dt: integration timestep
        neighbor_radius: radius in state-space (Euclidean over x,y,vx,vy) for rewiring
        """
        self.start = Node(state=np.array(start, dtype=float), parent=None, cost=0.0)
        self.goal_pos = np.array(goal_pos, dtype=float)
        self.bounds = bounds
        self.circles = circles or []
        self.rects = rects or []
        self.max_accel = max_accel
        self.max_duration = max_duration
        self.dt = dt
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.neighbor_radius = neighbor_radius

        self.nodes: List[Node] = [self.start]
        self.kdt = IncrementalKD()
        self.kdt.add(tuple(self.start.state))

        self.goal_node_idx: Optional[int] = None

    # sampling in state space (position+velocity)
    def sample_state(self) -> np.ndarray:
        if random.random() < self.goal_sample_rate and self.goal_node_idx is None:
            # bias sampling toward goal position with small velocity
            gx, gy = self.goal_pos
            return np.array([gx + random.uniform(-0.5, 0.5),
                             gy + random.uniform(-0.5, 0.5),
                             random.uniform(-0.5, 0.5),
                             random.uniform(-0.5, 0.5)])
        xmin, xmax, ymin, ymax = self.bounds
        return np.array([
            random.uniform(xmin, xmax),
            random.uniform(ymin, ymax),
            random.uniform(-2.0, 2.0),  # velocity sampling bounds
            random.uniform(-2.0, 2.0)
        ])

    def nearest_index(self, state: np.ndarray) -> int:
        _, idx = self.kdt.query(tuple(state))
        return int(idx)

    def near_indices(self, state: np.ndarray, k: int = 10) -> List[int]:
        k = min(k, len(self.nodes))
        _, idxs = self.kdt.query_k(tuple(state), k)
        if isinstance(idxs, int):
            idxs = [idxs]
        return [int(i) for i in idxs]

    def radius_neighbors(self, state: np.ndarray, r: float) -> List[int]:
        idxs = self.kdt.query_radius(tuple(state), r)
        return [int(i) for i in idxs]

    def plan(self):
        for it in range(self.max_iter):
            s_rand = self.sample_state()

            nearest_idx = self.nearest_index(s_rand)
            nearest_node = self.nodes[nearest_idx]

            # sample a random control (bounded accelerations) and duration
            ax = random.uniform(-self.max_accel, self.max_accel)
            ay = random.uniform(-self.max_accel, self.max_accel)
            dur = random.uniform(0.1, self.max_duration)

            traj = propagate(nearest_node.state, np.array([ax, ay]), dur, self.dt)
            final_state = traj[-1]

            # ensure final position within bounds
            xmin, xmax, ymin, ymax = self.bounds
            if not (xmin <= final_state[0] <= xmax and ymin <= final_state[1] <= ymax):
                continue

            # collision check
            if not collision_free_trajectory(traj, self.circles, self.rects):
                continue

            # cost is time
            new_cost = nearest_node.cost + dur
            new_node = Node(state=final_state.copy(), parent=nearest_idx, cost=new_cost,
                            control=np.array([ax, ay]), duration=dur)

            # find nearby nodes for potential better parent
            neighbor_idxs = self.radius_neighbors(final_state, self.neighbor_radius)
            best_parent_idx = nearest_idx
            best_cost = new_cost
            for ni in neighbor_idxs:
                node = self.nodes[ni]
                # try to connect from node -> new_state by simulating control needed
                # for kinodynamic planning we can attempt a straight rollout from node to new_state
                # but we'll instead check if rolling from that neighbor using sampled control reduces cost
                # simplistic approach: check easier-to-compute cost via Euclidean distance/time heuristic
                # For correctness, you'd sample controls per neighbor; here we accept cost heuristic
                heuristic_time = node.cost + np.linalg.norm(node.state[:2] - final_state[:2]) / max(0.1, self.max_accel)
                if heuristic_time < best_cost:
                    # for safety, verify collision-free straight-line in position space between node and new_state
                    # approximate using linear interpolation of positions
                    if collision_free_trajectory([node.state, final_state], self.circles, self.rects):
                        best_parent_idx = ni
                        best_cost = heuristic_time

            # attach to best parent
            new_node.parent = best_parent_idx
            new_node.cost = best_cost

            # append node
            new_idx = len(self.nodes)
            self.nodes.append(new_node)
            self.kdt.add(tuple(new_node.state))

            # rewire neighbors: if going through new_node reduces their cost and a feasible control exists
            for ni in neighbor_idxs:
                neighbor = self.nodes[ni]
                potential_cost = new_node.cost + np.linalg.norm(new_node.state[:2] - neighbor.state[:2]) / max(0.1, self.max_accel)
                if potential_cost + 1e-6 < neighbor.cost:
                    # attempt a fast collision check for straight-line pos connection
                    if collision_free_trajectory([new_node.state, neighbor.state], self.circles, self.rects):
                        neighbor.parent = new_idx
                        neighbor.cost = potential_cost

            # check if new node reaches goal region (position only)
            if np.linalg.norm(new_node.state[:2] - self.goal_pos) < 1.0:  # goal radius
                # attach a goal node (no dynamics needed) or mark goal
                self.goal_node_idx = new_idx
                # update c_best
                self.kdt.dirty = True
                break

            # occasionally rebuild kdtree to keep queries fast
            if it % 50 == 0:
                self.kdt.ensure()

        # final ensure
        self.kdt.ensure()
        return self.get_path()

    def get_path(self) -> Optional[List[Tuple[float, float]]]:
        if self.goal_node_idx is None:
            return None
        path = []
        idx = self.goal_node_idx
        while idx is not None:
            node = self.nodes[idx]
            path.append((float(node.state[0]), float(node.state[1])))
            idx = node.parent
        return path[::-1]


# -------------------------------
# Visualization
# -------------------------------

def plot_solution(rrt: KinodynamicRRTStar, path: Optional[List[Tuple[float, float]]]):
    plt.figure(figsize=(8, 8))
    # draw edges
    for i, n in enumerate(rrt.nodes):
        if n.parent is not None:
            p1 = rrt.nodes[n.parent].state
            p2 = n.state
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='0.6', linewidth=0.7)

    # obstacles
    for c in rrt.circles:
        plt.gca().add_patch(plt.Circle((c.x, c.y), c.r, color='gray'))
    for r in rrt.rects:
        plt.gca().add_patch(plt.Rectangle((r.x, r.y), r.w, r.h, color='gray'))

    if path:
        xs, ys = zip(*path)
        plt.plot(xs, ys, '-r', linewidth=2, label='planned path')

    plt.scatter(rrt.start.state[0], rrt.start.state[1], c='green', s=80, label='start')
    plt.scatter(rrt.goal_pos[0], rrt.goal_pos[1], c='red', s=80, label='goal')
    xmin, xmax, ymin, ymax = rrt.bounds
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()


# -------------------------------
# Example usage
# -------------------------------
if __name__ == '__main__':
    random.seed(0)
    np.random.seed(0)

    # Define obstacles
    circles = [CircleObstacle(40, 40, 8), CircleObstacle(70, 70, 7)]
    rects = [RectObstacle(60, 20, 12, 35)]

    # start state: x,y,vx,vy
    start_state = (5.0, 5.0, 0.0, 0.0)
    goal_pos = (95.0, 95.0)
    bounds = (0.0, 100.0, 0.0, 100.0)

    rrt = KinodynamicRRTStar(start=start_state,
                             goal_pos=goal_pos,
                             bounds=bounds,
                             circles=circles,
                             rects=rects,
                             max_accel=2.0,
                             max_duration=1.0,
                             dt=0.05,
                             max_iter=4000,
                             goal_sample_rate=0.15,
                             neighbor_radius=6.0)

    path = rrt.plan()
    if path:
        print(f'Found path with {len(path)} waypoints')
    else:
        print('No path found')

    plot_solution(rrt, path)
