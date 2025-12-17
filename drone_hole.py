import time
import numpy as np
import pybullet as p

from make_wall import spawn_wall_with_hole, create_ceiling
from kino_dynamic_planner import rrt
from check_point import draw_debug_point
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
import trimesh
import numpy as np
def run_sim(walls = 4,  # Number of walls to create
    max_wall_dist = 2,
    min_wall_dist = 1,
    thickness = 0.1,
    wall_height = 2.0,
    wall_width = 1.0,
    hole_width =0.5,
    hole_height = 0.7,
    padding = 0.1):
    #### Create environment ####################################################
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 0.5]]),
        physics=Physics.PYB,
        gui=True,
        record=False,
        obstacles=False,
        user_debug_gui = False
    )
    #### Create PID controller #################################################
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    #### Reset environment #####################################################
    obs, _ = env.reset()



    #### create ceiling and walls
    body_ids = []
    for wall in range(walls):
        for body_id in (spawn_wall_with_hole(
            hole_width=hole_width,
            hole_height=hole_height,
            wall_width=wall_width,
            wall_height=wall_height,
            thickness=thickness,
            center=(np.random.rand(),np.random.rand()),
            base_position=(0,wall*(max_wall_dist)+np.random.uniform(min_wall_dist, max_wall_dist),1))):
            body_ids.append(body_id)
        # Load walls 
    for body_id in (create_ceiling(thickness, walls, max_wall_dist, wall_height, min_wall_dist, wall_width)):
        body_ids.append(body_id)
    print(body_ids)
    #### Hover target position #################################################
    target_pos = np.array([0.0, 0.0, 1.0])
    target_rpy = np.array([0.0, 0.0, 0.0])

    action = np.zeros((1, 4))

    #### Create rrt path #######################################################    
    draw_debug_point([0, walls*max_wall_dist+1, wall_height/2], size = 10, color=[2,2,2])
    path = rrt(              start=[0,0,1,0,0,0],
                             goal=[0, walls*max_wall_dist+1, wall_height/2],
                             bounds=[(-wall_width/2,wall_width/2),(-1,walls*max_wall_dist),(0,wall_height)],
                             obstacles = body_ids,
                             padding = padding)
    print(path)
    # assert path != None, "could not find path"
    path_xyz = path[:, :3]   # Extract only positions (x,y,z)
    for i in range(len(path_xyz) - 1):
        p1 = path_xyz[i]
        p2 = path_xyz[i + 1]
        p.addUserDebugLine(p1, p2, [0, 1, 0], lineWidth=3)
    #### Run simulation ########################################################
    # Follow the RRT path using interpolation
    for i in range(len(path) - 1):

        p0 = path[i, :3]      # start point of segment
        p1 = path[i+1, :3]    # end point of segment

        # Generate 100 interpolated positions between p0 and p1
        for t_step in range(100):
            alpha = t_step / 100.0
            target_pos = (1 - alpha) * p0 + alpha * p1
            
            # Keep fixed yaw
            target_rpy = np.array([0.0, 0.0, 0.0])

            state = obs[0]

            # Compute PID control
            rpm, _, _ = ctrl.computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=state,
                target_pos=target_pos,
                target_rpy=target_rpy
            )
            action[0, :] = rpm

            obs, reward, terminated, truncated, info = env.step(action)
            env.render()
            time.sleep(1.0 / env.PYB_FREQ)

            if terminated or truncated:
                break


    try:
        p.removeAllUserDebugItems()
    except Exception:
        pass

    time.sleep(0.1)

    # Explicitly disconnect pybullet (only if you're connected directly)
    env.close()
    # extra short pause then exit
    return info, reward
if __name__ == "__main__":
    #### Settings ##############################################################
    run_sim(walls = 4,
    max_wall_dist = 2,
    min_wall_dist = 1,
    thickness = 0.1,
    wall_height = 2.0,
    wall_width = 1.0,
    hole_width =0.5,
    hole_height = 0.7,
    padding = 0.1)



    
