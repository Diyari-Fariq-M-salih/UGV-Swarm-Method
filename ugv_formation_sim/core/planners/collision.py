import math

def point_in_circle(px, py, cx, cy, r):
    return math.hypot(px - cx, py - cy) <= r

def point_in_rect(px, py, x1, y1, x2, y2):
    return (x1 <= px <= x2) and (y1 <= py <= y2)

def collision_point(point, obstacles):
    """Check if a point collides with ANY obstacle"""
    px, py = point
    for obs in obstacles:
        if obs[0] == "circle":
            _, cx, cy, r = obs
            if point_in_circle(px, py, cx, cy, r):
                return True

        elif obs[0] == "rect":
            _, x1, y1, x2, y2 = obs
            if point_in_rect(px, py, x1, y1, x2, y2):
                return True

    return False


def collision_segment(p1, p2, obstacles, step=0.05):
    """Check if line segment collides with any obstacle"""
    x1, y1 = p1
    x2, y2 = p2

    dist = math.hypot(x2 - x1, y2 - y1)
    if dist == 0:
        return collision_point(p1, obstacles)

    steps = int(dist / step)

    for i in range(steps + 1):
        u = i / steps
        x = (1 - u) * x1 + u * x2
        y = (1 - u) * y1 + u * y2
        if collision_point((x, y), obstacles):
            return True

    return False
