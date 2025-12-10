import pybullet as p

def is_point_in_obstacle(point, obstacle_ids):
    for body in obstacle_ids:
        aabb_min, aabb_max = p.getAABB(body)
        if point_in_aabb(point, aabb_min, aabb_max):
            return True
    return False
def point_in_aabb(point, aabb_min, aabb_max):
    return (aabb_min[0] <= point[0] <= aabb_max[0] and
            aabb_min[1] <= point[1] <= aabb_max[1] and
            aabb_min[2] <= point[2] <= aabb_max[2])