import pybullet as p

def is_point_in_obstacle(point, obstacle_ids, padding):
    for body in obstacle_ids:
        aabb_min, aabb_max = p.getAABB(body)
        if point_in_aabb(point, aabb_min, aabb_max, padding):
            return True
    return False
def point_in_aabb(point, aabb_min, aabb_max, padding=0.15):
    return (aabb_min[0]-padding <= point[0] <= aabb_max[0]+padding and
            aabb_min[1]-padding <= point[1] <= aabb_max[1]+padding and
            aabb_min[2]-padding <= point[2] <= aabb_max[2]+padding)
def draw_debug_point(pnt, color=[1,0,0], size=0.05):
    x, y, z = pnt
    s = size
    p.addUserDebugLine([x-s, y, z], [x+s, y, z], color)
    p.addUserDebugLine([x, y-s, z], [x, y+s, z], color)
    p.addUserDebugLine([x, y, z-s], [x, y, z+s], color)

