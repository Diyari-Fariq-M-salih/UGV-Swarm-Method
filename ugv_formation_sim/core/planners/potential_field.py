import math
from .planner_base import PathPlanner

class PotentialFieldPlanner(PathPlanner):
    def __init__(self, step=0.2, alpha=1.0, beta=100.0, obs_radius=1.0):
        self.step = step
        self.alpha = alpha  # attractive gain
        self.beta = beta    # repulsive gain
        self.obs_radius = obs_radius

    def plan(self, start, goal, obstacles):
        x, y = start
        gx, gy = goal
        path = [(x, y)]

        for _ in range(1000):
            # Attractive force
            fx = self.alpha * (gx - x)
            fy = self.alpha * (gy - y)

            # Repulsive force
            for ox, oy, rad in obstacles:
                d = math.hypot(x-ox, y-oy)
                if d < self.obs_radius + rad:
                    f = self.beta * (1/d - 1/(self.obs_radius+rad)) / (d**3)
                    fx += f * (x - ox)
                    fy += f * (y - oy)

            # Move along gradient
            x += self.step * fx
            y += self.step * fy

            path.append((x, y))

            # Goal check
            if math.hypot(x-gx, y-gy) < 0.5:
                break

        return path
