import numpy as np
import heapq

# =============================
# 3D AABB Obstacle
# =============================
class AABB:
    def __init__(self, min_corner, max_corner):
        self.min = np.array(min_corner)
        self.max = np.array(max_corner)

    def contains(self, p):
        return np.all(p >= self.min) and np.all(p <= self.max)

# =============================
# Collision Checking for Trajectory
# =============================
def trajectory_collides(traj, obstacles):
    """Check collision for a trajectory (Nx6 array of states or Nx3 positions).

    The trajectory may be:
    - array of states shape (N,6) where columns 0..2 are positions, or
    - array of positions shape (N,3).
    """
    if traj.size == 0:
        return False
    arr = np.asarray(traj)
    if arr.ndim != 2:
        raise ValueError("traj must be a 2D array")
    if arr.shape[1] == 6:
        points = arr[:, :3]
    elif arr.shape[1] == 3:
        points = arr
    else:
        raise ValueError("traj rows must have length 3 (pos) or 6 (state)")

    for p in points:
        for obs in obstacles:
            if obs.contains(p):
                return True
    return False

# =============================
# State Propagation (3D Kinodynamic)
# =============================
def propagate(state, control, dt=0.05, steps=20):
    """Propagate a full 6D state under constant control (ax,ay,az).

    state: array-like length 6: [x,y,z,vx,vy,vz]
    control: array-like length 3: [ax,ay,az]
    returns (trajectory as (steps,6) array, final_state (6,))
    """
    traj = []
    s = np.array(state, dtype=float)
    ax, ay, az = control
    for _ in range(steps):
        px, py, pz, vx, vy, vz = s

        # update velocity
        vx2 = vx + ax * dt
        vy2 = vy + ay * dt
        vz2 = vz + az * dt

        # update position (kinematic eqns)
        px2 = px + vx * dt + 0.5 * ax * dt * dt
        py2 = py + vy * dt + 0.5 * ay * dt * dt
        pz2 = pz + vz * dt + 0.5 * az * dt * dt

        s = np.array([px2, py2, pz2, vx2, vy2, vz2])
        traj.append(s)

    return np.array(traj), s

# =============================
# Distance Metric (robust to 3D pos or 6D state)
# =============================

def make_full_state(x):
    """Convert a 3-element position or 6-element state into a full 6D state.

    - If x has length 3 → [x,y,z, 0,0,0]
    - If x has length 6 → returned as np.array
    """
    a = np.asarray(x)
    if a.size == 3:
        return np.array([a[0], a[1], a[2], 0.0, 0.0, 0.0], dtype=float)
    if a.size == 6:
        return a.astype(float)
    raise ValueError("Input must be length 3 (position) or 6 (state)")


def distance(s1, s2, alpha=1.0, beta=0.3):
    """Distance metric between states or positions.

    Accepts inputs that are length-3 (positions) or length-6 (state). Internally
    compares position and velocity parts with weights.
    """
    a = make_full_state(s1)
    b = make_full_state(s2)
    dp = np.linalg.norm(a[:3] - b[:3])
    dv = np.linalg.norm(a[3:] - b[3:])
    return alpha * dp + beta * dv

# =============================
# Random Control Sampling
# =============================

def sample_random_control(a_max=4.0):
    return np.random.uniform(-a_max, a_max, size=3)

# =============================
# RRT Node
# =============================
class Node:
    def __init__(self, state, parent=None, control=None):
        self.state = np.array(state, dtype=float)
        self.parent = parent
        self.control = control

# =============================
# Find Nearest Node (robust)
# =============================
def nearest(nodes, sample):
    """Return the nearest node object (by distance) to the sample.

    sample may be a 3D position or a full 6D state.
    """
    s_full = make_full_state(sample)
    best = None
    best_d = float('inf')
    for n in nodes:
        d = distance(n.state, s_full)
        if d < best_d:
            best = n
            best_d = d
    return best

# =============================
# Reconstruct Final Path
# =============================

def reconstruct_path(node):
    path = []
    while node is not None:
        path.append(node.state)
        node = node.parent
    return np.array(path[::-1])

# =============================
# Main RRT Planning Loop
# =============================

def rrt(start, goal, obstacles, bounds, max_iter=5000, goal_thresh=1.0):
    """RRT planner in 3D kinodynamic state-space.

    start: length-6 state [x,y,z,vx,vy,vz]
    goal: length-3 or length-6 (if full state) target
    bounds: ([xmin,xmax],[ymin,ymax],[zmin,zmax]) or flat tuple
    """
    # normalize bounds to flat tuple (xmin,xmax,ymin,ymax,zmin,zmax)
    if len(bounds) == 3 and all(len(b) == 2 for b in bounds):
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounds
    else:
        xmin, xmax, ymin, ymax, zmin, zmax = bounds

    nodes = [Node(start)]

    for it in range(max_iter):
        # occasionally bias toward goal
        if np.random.rand() < 0.15:
            sample = goal
        else:
            sample = np.array([np.random.uniform(xmin, xmax),
                               np.random.uniform(ymin, ymax),
                               np.random.uniform(zmin, zmax)])

        near = nearest(nodes, sample)
        control = sample_random_control()
        traj, new_state = propagate(near.state, control)

        if trajectory_collides(traj, obstacles):
            continue

        new_node = Node(new_state, parent=near, control=control)
        nodes.append(new_node)

        # check goal proximity (compare positions)
        if distance(new_state, goal) < goal_thresh:
            print("Goal reached at iteration", it)
            return reconstruct_path(new_node)

    print("Failed to find a path.")
    return None

# =============================
# Example Usage
# =============================
if __name__ == "__main__":
    np.random.seed(0)

    start = np.array([0, 0, 1, 0, 0, 0])  # x,y,z,vx,vy,vz
    goal = np.array([10, 10, 2])  # position-only goal

    obstacles = [
        AABB([3, 3, 0], [5, 5, 3]),
        AABB([7, 7, 0], [8, 8, 5])
    ]

    bounds = ((-5, 15), (-5, 15), (0, 6))

    path = rrt(start, goal, obstacles, bounds)

    if path is not None:
        print("Path (states):")
        print(path)
