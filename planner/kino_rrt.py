#!/usr/bin/env python3
"""
Kinodynamic Optimal RRT* for Quadcopter in 3D
- State-space RRT* with quadcopter dynamics
- Bounded acceleration/velocity constraints
- Optimized collision checking with early termination
- Maintains bounds checking throughout trajectory
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
from scipy.integrate import odeint
import math
import random
from collections import namedtuple

# Type definitions for clarity
State = namedtuple('State', ['x', 'y', 'z', 'vx', 'vy', 'vz'])

# Quadcopter dynamics parameters
class QuadcopterParams:
    """Quadcopter physical parameters."""
    def __init__(self):
        self.mass = 1.0  # kg
        self.gravity = 9.81  # m/s^2
        self.max_thrust = 2.0 * self.mass * self.gravity  # N (max 2g acceleration)
        self.max_velocity = 10.0  # m/s
        self.max_acceleration = 2.0 * self.gravity  # m/s^2
        
    def get_max_accel(self):
        return self.max_acceleration
    
    def get_max_velocity(self):
        return self.max_velocity


# Fast collision checking functions
def is_collision_free_point(point, obstacles):
    """
    Check if a single point is collision-free.
    Optimized for fast early termination.
    """
    for obs in obstacles:
        obs_type = obs.get("type", "sphere")
        
        if obs_type == "sphere":
            center = (obs["x"], obs["y"], obs["z"])
            dist_sq = sum((point[i] - center[i]) ** 2 for i in range(3))
            if dist_sq <= obs["r"] ** 2:
                return False
                
        elif obs_type == "box":
            # AABB test (fastest for boxes)
            if (obs["x"] <= point[0] <= obs["x"] + obs["w"] and
                obs["y"] <= point[1] <= obs["y"] + obs["h"] and
                obs["z"] <= point[2] <= obs["z"] + obs["d"]):
                return False
    
    return True


def is_collision_free_segment(p1, p2, obstacles, resolution=15):
    """
    Check if path segment between p1 and p2 is collision-free.
    Uses optimized linear interpolation with early termination.
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    delta = p2 - p1
    
    # Quick AABB check first to skip obvious collisions
    min_coords = np.minimum(p1, p2)
    max_coords = np.maximum(p1, p2)
    
    for obs in obstacles:
        obs_type = obs.get("type", "sphere")
        
        if obs_type == "sphere":
            # Test closest point on segment to sphere
            center = np.array([obs["x"], obs["y"], obs["z"]])
            radius = obs["r"]
            
            # Project center onto line segment
            t = np.dot(center - p1, delta) / np.dot(delta, delta)
    

    def add(self, point):
        """Query k nearest neighbors."""
            #!/usr/bin/env python3
            """
            Minimum-snap trajectory generator and flatness mapping for a quadcopter (3D).

            This implements the core flatness-based mapping proposed in the quadcopter
            minimum-snap literature: trajectories are generated in the flat outputs
            (x,y,z, yaw) via polynomials; accelerations and higher derivatives are
            computed and then mapped to collective thrust and desired orientation.

            This file provides:
            - A simple per-segment 7th-order polynomial (minimum-snap for a single
              segment with boundary conditions on position..jerk).
            - Polynomial coefficient solver and evaluation helpers (pos, vel, acc, jerk, snap).
            - Flatness mapping: from desired accel+yaw -> thrust and desired rotation matrix
            - Small example showing usage and plotting.

            Notes:
            - For multi-segment minimum-snap with continuity across segments a QP
              formulation is typical; here we implement per-segment degree-7
              polynomials which satisfy 8 boundary constraints (pos, vel, acc, jerk
              at start and end). This matches the per-segment closed-form approach
              from the literature and is suitable for many waypoint-based pipelines.
            """

            from typing import List, Tuple
            import numpy as np
            import math
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D


            # -----------------------------
            # Polynomial utilities (degree-7)
            # -----------------------------

            def poly_coeff_matrix(t0: float, tf: float) -> np.ndarray:
                """Build the 8x8 constraint matrix for a degree-7 polynomial on [t0,tf].

                Constraints order: p(t0), v(t0), a(t0), j(t0), p(tf), v(tf), a(tf), j(tf)
                Polynomial basis: a0 + a1 t + a2 t^2 + ... + a7 t^7
                Returns matrix A where A @ coeffs = b (b are boundary values)
                """
                T0 = np.array([t0**i for i in range(8)])
                T0_d1 = np.array([0 if i == 0 else i * t0**(i-1) for i in range(8)])
                T0_d2 = np.array([0 if i < 2 else i*(i-1)*t0**(i-2) for i in range(8)])
                T0_d3 = np.array([0 if i < 3 else i*(i-1)*(i-2)*t0**(i-3) for i in range(8)])

                TF = np.array([tf**i for i in range(8)])
                TF_d1 = np.array([0 if i == 0 else i * tf**(i-1) for i in range(8)])
                TF_d2 = np.array([0 if i < 2 else i*(i-1)*tf**(i-2) for i in range(8)])
                TF_d3 = np.array([0 if i < 3 else i*(i-1)*(i-2)*tf**(i-3) for i in range(8)])

                A = np.vstack([T0, T0_d1, T0_d2, T0_d3, TF, TF_d1, TF_d2, TF_d3])
                return A


            def solve_polynomial_segment(p0, v0, a0, j0, pf, vf, af, jf, t0=0.0, tf=1.0):
                """Solve for degree-7 polynomial coefficients for a single segment.

                Inputs are scalars for single axis. Returns coeffs (length 8).
                """
                A = poly_coeff_matrix(t0, tf)
                b = np.array([p0, v0, a0, j0, pf, vf, af, jf], dtype=float)
                coeffs = np.linalg.solve(A, b)
                return coeffs


            def eval_poly(coeffs: np.ndarray, t: float) -> Tuple[float, float, float, float, float]:
                """Evaluate polynomial and derivatives at time t.

                Returns (pos, vel, acc, jerk, snap) where snap is 4th derivative.
                """
                # Precompute powers
                powers = np.array([t**i for i in range(len(coeffs))])
                pos = float(np.dot(coeffs, powers))

                # First derivative
                d1_coefs = np.array([i * coeffs[i] for i in range(len(coeffs))])
                vel = float(np.dot(d1_coefs[1:], powers[:-1])) if len(coeffs) > 1 else 0.0

                # Second derivative
                d2_coefs = np.array([i*(i-1)*coeffs[i] for i in range(len(coeffs))])
                acc = float(np.dot(d2_coefs[2:], powers[:-2])) if len(coeffs) > 2 else 0.0

                # Third derivative (jerk)
                d3_coefs = np.array([i*(i-1)*(i-2)*coeffs[i] for i in range(len(coeffs))])
                jerk = float(np.dot(d3_coefs[3:], powers[:-3])) if len(coeffs) > 3 else 0.0

                # Fourth derivative (snap)
                d4_coefs = np.array([i*(i-1)*(i-2)*(i-3)*coeffs[i] for i in range(len(coeffs))])
                snap = float(np.dot(d4_coefs[4:], powers[:-4])) if len(coeffs) > 4 else 0.0

                return pos, vel, acc, jerk, snap


            # -----------------------------
            # Flatness mapping: accel -> thrust + desired orientation
            # -----------------------------

            def accel_to_thrust_and_rotation(acceleration: np.ndarray, yaw: float, mass: float = 1.0, gravity: float = 9.81):
                """Map desired acceleration and yaw to total thrust and desired rotation matrix.

                Args:
                    acceleration: desired inertial acceleration vector (3,)
                    yaw: desired yaw angle (rad)
                    mass: quadcopter mass
                Returns:
                    thrust: scalar collective thrust (N)
                    R: 3x3 desired rotation matrix from body -> world
                """
                # Desired force = m * (acc + g e3)
                e3 = np.array([0.0, 0.0, 1.0])
                F = mass * (acceleration + gravity * e3)
                F_norm = np.linalg.norm(F)
                if F_norm < 1e-6:
                    # avoid division by zero -- hover
                    zb = np.array([0.0, 0.0, 1.0])
                else:
                    zb = F / F_norm

                # Desired heading in world frame
                psi = yaw
                xc = np.array([math.cos(psi), math.sin(psi), 0.0])

                # Compute yb and xb
                yb = np.cross(zb, xc)
                yb_norm = np.linalg.norm(yb)
                if yb_norm < 1e-6:
                    # if zb nearly aligned with xc, choose arbitrary orthogonal vector
                    if abs(zb[2]) < 0.99:
                        yb = np.cross(zb, np.array([0.0, 0.0, 1.0]))
                    else:
                        yb = np.cross(zb, np.array([0.0, 1.0, 0.0]))
                    yb_norm = np.linalg.norm(yb)
                yb = yb / yb_norm
                xb = np.cross(yb, zb)

                R = np.column_stack((xb, yb, zb))
                thrust = F_norm
                return thrust, R


            def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
                """Convert rotation matrix to roll, pitch, yaw (ZYX convention).

                Returns (phi, theta, psi)
                """
                # Prevent numerical issues
                sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
                singular = sy < 1e-6
                if not singular:
                    x = math.atan2(R[2, 1], R[2, 2])
                    y = math.atan2(-R[2, 0], sy)
                    z = math.atan2(R[1, 0], R[0, 0])
                else:
                    x = math.atan2(-R[1, 2], R[1, 1])
                    y = math.atan2(-R[2, 0], sy)
                    z = 0.0
                return x, y, z


            # -----------------------------
            # Example: produce a minimum-snap segment and map to thrust+attitude
            # -----------------------------

            def example_single_segment():
                # Create a single segment from start -> goal in 5 seconds
                t0 = 0.0
                tf = 5.0

                # Boundary conditions for x,y,z (pos, vel, acc, jerk)
                p0 = np.array([0.0, 0.0, 0.0])
                v0 = np.array([0.0, 0.0, 0.0])
                a0 = np.array([0.0, 0.0, 0.0])
                j0 = np.array([0.0, 0.0, 0.0])

                pf = np.array([10.0, 5.0, 3.0])
                vf = np.array([0.0, 0.0, 0.0])
                af = np.array([0.0, 0.0, 0.0])
                jf = np.array([0.0, 0.0, 0.0])

                coeffs_xyz = []
                for dim in range(3):
                    coeffs = solve_polynomial_segment(
                        p0[dim], v0[dim], a0[dim], j0[dim],
                        pf[dim], vf[dim], af[dim], jf[dim], t0=t0, tf=tf
                    )
                    coeffs_xyz.append(coeffs)

                # Sample trajectory and map to thrust+orientation
                times = np.linspace(t0, tf, 201)
                positions = []
                thrusts = []
                eulers = []
                yaw_des = 0.0
                mass = 1.0
                for t in times:
                    pos = []
                    vel = []
                    acc = []
                    for coeffs in coeffs_xyz:
                        p, v, a, _, _ = eval_poly(coeffs, t)
                        pos.append(p)
                        vel.append(v)
                        acc.append(a)
                    pos = np.array(pos)
                    acc = np.array(acc)

                    thrust, R = accel_to_thrust_and_rotation(acc, yaw_des, mass=mass)
                    euler = rotation_matrix_to_euler(R)

                    positions.append(tuple(pos))
                    thrusts.append(thrust)
                    eulers.append(euler)

                # Plot trajectory
                fig = plt.figure(figsize=(10, 6))
                ax = fig.add_subplot(121, projection='3d')
                xs, ys, zs = zip(*positions)
                ax.plot(xs, ys, zs, '-b')
                ax.scatter(p0[0], p0[1], p0[2], c='g', s=80, label='start')
                ax.scatter(pf[0], pf[1], pf[2], c='r', s=80, label='goal')
                ax.set_title('Minimum-snap segment (deg7)')
                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                ax.legend()

                ax2 = fig.add_subplot(222)
                ax2.plot(times, thrusts)
                ax2.set_title('Collective thrust (N)')
                ax2.set_xlabel('t [s]')

                ax3 = fig.add_subplot(224)
                rolls = [e[0] for e in eulers]
                pitches = [e[1] for e in eulers]
                yaws = [e[2] for e in eulers]
                ax3.plot(times, rolls, label='roll')
                ax3.plot(times, pitches, label='pitch')
                ax3.plot(times, yaws, label='yaw')
                ax3.set_title('Euler angles (rad)')
                ax3.legend()

                plt.tight_layout()
                plt.show()


            if __name__ == '__main__':
                example_single_segment()
    def k_nearest_neighbors(self, state):
