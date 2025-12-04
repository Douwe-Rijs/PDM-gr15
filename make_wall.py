import pybullet as p
import numpy as np

def spawn_wall_with_hole(
    wall_width=2.0,
    wall_height=2.0,
    thickness=0.1,
    hole_width=0.5,
    hole_height=0.5,
    center=(0.5, 0.5),      # NORMALIZED (0..1) hole center in the wall
    base_position=(0,0,0)
):
    objs = []
    
    W = wall_width
    H = wall_height
    w = hole_width
    h = hole_height
    margin_x = w / W / 2    # half hole width in normalized space
    margin_z = h / H / 2    # half hole height in normalized space

    cx = np.clip(center[0], margin_x, 1 - margin_x)
    cz = np.clip(center[1], margin_z, 1 - margin_z)
    # Wall bounds (centered at 0,0)
    left_bound  = -W/2
    right_bound = +W/2
    bot         = -H/2
    top         = +H/2

    # Convert normalized coordinates to local wall coordinates
    hole_center_x = left_bound + cx * W
    hole_center_z = bot + cz * H

    # hole edges
    hole_left   = hole_center_x - w/2
    hole_right  = hole_center_x + w/2
    hole_bottom = hole_center_z - h/2
    hole_top    = hole_center_z + h/2

    segments = []

    # LEFT segment
    left_width = hole_left - left_bound
    left_center_x = (hole_left + left_bound) / 2
    left_half = [left_width/2, thickness/2, H/2]
    segments.append(([left_center_x, 0, 0], left_half))

    # RIGHT segment
    right_width = right_bound - hole_right
    right_center_x = (hole_right + right_bound) / 2
    right_half = [right_width/2, thickness/2, H/2]
    segments.append(([right_center_x, 0, 0], right_half))

    # BOTTOM segment
    bottom_height = hole_bottom - bot
    bottom_center_z = (hole_bottom + bot) / 2
    bottom_half = [w/2, thickness/2, bottom_height/2]
    segments.append(([hole_center_x, 0, bottom_center_z], bottom_half))

    # TOP segment
    top_height = top - hole_top
    top_center_z = (hole_top + top) / 2
    top_half = [w/2, thickness/2, top_height/2]
    segments.append(([hole_center_x, 0, top_center_z], top_half))

    # spawn in world
    bx, by, bz = base_position
    body_ids = []

    for pos_local, half_ext in segments:
        pos_world = [
            pos_local[0] + bx,
            pos_local[1] + by,
            pos_local[2] + bz
        ]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_ext)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_ext, rgbaColor=[1,1,1,1])
        body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=pos_world
        )
        body_ids.append(body)

    return body_ids

def create_ceiling(thickness, walls, max_wall_dist, wall_height, min_wall_dist, wall_width):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thickness, walls*max_wall_dist, wall_height/2])
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[thickness, walls*max_wall_dist, wall_height/2],
        rgbaColor=[1,1,1,0.1]    # transparent white
    )
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[wall_width/2+thickness,min_wall_dist,wall_height/2]
    )
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[-(wall_width/2+thickness),min_wall_dist,wall_height/2]
    )
    return
