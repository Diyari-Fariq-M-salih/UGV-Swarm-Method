import heapq
import numpy as np
import time
import math

class AStarPlanner:

    def __init__(self):
        self.path = []
        self.current_wp = 0

        # Metrics
        self.expanded_count = 0
        self.plan_time = 0.0
        self.path_length = 0.0

    def neighbors(self, gx, gy, env):
        # 8-connected grid
        moves = [
            (1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0),
            (1,1,math.sqrt(2)), (-1,1,math.sqrt(2)),
            (1,-1,math.sqrt(2)), (-1,-1,math.sqrt(2))
        ]
        for dx, dy, cost in moves:
            nx, ny = gx + dx, gy + dy
            if env.get_occupancy_grid(nx, ny) == 0:
                yield nx, ny, cost

    def plan(self, start, goal, env):
        t0 = time.time()

        sx, sy = env.world_to_grid(*start)
        gx, gy = env.world_to_grid(*goal)

        pq = []
        heapq.heappush(pq, (0, (sx, sy)))
        came_from = {}
        cost = {(sx, sy): 0}

        self.expanded_count = 0

        while pq:
            _, (x, y) = heapq.heappop(pq)
            self.expanded_count += 1

            if (x, y) == (gx, gy):
                break

            for nx, ny, step_cost in self.neighbors(x, y, env):
                new_cost = cost[(x, y)] + step_cost
                if (nx, ny) not in cost or new_cost < cost[(nx, ny)]:
                    cost[(nx, ny)] = new_cost

                    # Chebyshev heuristic (works with diagonals)
                    dx = abs(nx - gx)
                    dy = abs(ny - gy)
                    heuristic = math.sqrt(2)*min(dx, dy) + abs(dx - dy)

                    priority = new_cost + heuristic
                    heapq.heappush(pq, (priority, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)

        if (gx, gy) not in came_from:
            self.path = []
            return []

        # reconstruct grid path
        path_grid = [(gx, gy)]
        while path_grid[-1] != (sx, sy):
            path_grid.append(came_from[path_grid[-1]])
        path_grid.reverse()

        self.path = [env.grid_to_world(px, py) for px, py in path_grid]
        self.current_wp = 0

        self.plan_time = (time.time() - t0) * 1000.0
        self.path_length = sum(
            np.linalg.norm(np.array(self.path[i]) - np.array(self.path[i+1]))
            for i in range(len(self.path)-1)
        )

        return self.path

    def compute_force(self, pos, goal, env, others=None):
        # NO replanning in force computation
        if not self.path:
            return np.zeros(2), {}

        wp = np.array(self.path[self.current_wp])
        diff = wp - pos
        dist = np.linalg.norm(diff)

        if dist < 0.6 and self.current_wp < len(self.path)-1:
            self.current_wp += 1

        if dist < 1e-6:
            return np.zeros(2), {}

        return diff / dist * 4.0, {}
