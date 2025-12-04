import time
import numpy as np
import pybullet as p

from make_wall import spawn_wall_with_hole, create_ceiling
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
import trimesh
import numpy as np

if __name__ == "__main__":
    #### Settings ##############################################################
    walls = 10  # Number of walls to create
    max_wall_dist = 1
    min_wall_dist = 0.1
    thickness = 0.1
    wall_height = 2.0
    wall_width = 1.0
    hole_width =0.2
    hole_height = 0.2



    #### Create environment ####################################################
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 0.5]]),
        physics=Physics.PYB,
        gui=True,
        record=False,
        obstacles=False,
    )
    #### Create PID controller #################################################
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    #### Reset environment #####################################################
    obs, _ = env.reset()



    #### create ceiling and walls
    
    for wall in range(walls):
        spawn_wall_with_hole(
            hole_width=hole_width,
            hole_height=hole_height,
            wall_width=wall_width,
            wall_height=wall_height,
            thickness=thickness,
            center=(np.random.rand(),np.random.rand()),
            base_position=(0,wall*(max_wall_dist)+np.random.uniform(min_wall_dist, max_wall_dist),1))
        # Load walls 
    create_ceiling(thickness, walls, max_wall_dist, wall_height, min_wall_dist, wall_width)

    #### Hover target position #################################################
    target_pos = np.array([0.0, 0.0, 1.0])
    target_rpy = np.array([0.0, 0.0, 0.0])

    action = np.zeros((1, 4))
    #### Run simulation ########################################################
    while True:
        t = time.time()
        target_pos[0] = 0.5 * np.sin(1 * t)
        target_pos[1] = 0.5 * np.cos(1 * t)
        state = obs[0]   # 1D array of length 20

        # Compute PID motor RPMs
        rpm, _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=state,
            target_pos=target_pos,
            target_rpy=target_rpy
        )

        action[0, :] = rpm

        # Step simulation
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()

        time.sleep(1.0 / env.PYB_FREQ)

        if terminated or truncated:
            break

    env.close()
