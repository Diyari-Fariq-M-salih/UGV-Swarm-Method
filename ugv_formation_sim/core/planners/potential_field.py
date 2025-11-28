import math
from .planner_base import PathPlanner
from .collision import *

class PotentialFieldPlanner(PathPlanner):
    def __init__(self, step=0.1, alpha=1.0, beta=200.0, obs_radius=1.5):
        self.step = step
        self.alpha = alpha  # attraction
        self.beta = beta    # repulsion strength
        self.obs_radius = obs_radius

    def plan(self, start, goal, obstacles):
        x, y = start
        gx, gy = goal
        path = [(x, y)]

        for _ in range(2000):
            # Attractive force
            fx = self.alpha * (gx - x)
            fy = self.alpha * (gy - y)

            # Repulsive force from obstacles
            for obs in obstacles:

                if obs[0] == "circle":
                    _, ox, oy, r = obs
                    # closest point is center
                    d = math.hypot(x - ox, y - oy)
                    if d < r + self.obs_radius:
                        f = self.beta * (1/d - 1/(r + self.obs_radius)) / (d**3)
                        fx += f * (x - ox)
                        fy += f * (y - oy)

                elif obs[0] == "rect":
                    _, x1, y1, x2, y2 = obs

                    # closest point on rectangle to robot
                    cx = max(x1, min(x, x2))
                    cy = max(y1, min(y, y2))

                    d = math.hypot(x - cx, y - cy)
                    if d < self.obs_radius:
                        f = self.beta * (1/d - 1/self.obs_radius) / (d**3)
                        fx += f * (x - cx)
                        fy += f * (y - cy)

            # Move robot along force field
            x += self.step * fx
            y += self.step * fy
            path.append((x, y))

            # Goal reached
            if math.hypot(x - gx, y - gy) < 0.3:
                break

        return path
