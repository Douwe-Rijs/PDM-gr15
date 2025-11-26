import time
import numpy as np

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

if __name__ == "__main__":

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
