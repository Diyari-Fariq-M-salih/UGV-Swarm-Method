import heapq
import numpy as np

class AStarPlanner:

    def __init__(self):
        self.path = []
        self.current_wp = 0

    def neighbors(self, gx, gy, env):
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy
            if env.get_occupancy_grid(nx, ny) == 0:
                yield nx, ny

    def plan(self, start, goal, env):
        sx, sy = env.world_to_grid(*start)
        gx, gy = env.world_to_grid(*goal)

        pq = []
        heapq.heappush(pq, (0, (sx, sy)))
        came_from = {}
        cost = {(sx, sy): 0}

        while pq:
            _, (x, y) = heapq.heappop(pq)
            if (x, y) == (gx, gy):
                break

            for nx, ny in self.neighbors(x, y, env):
                new_cost = cost[(x, y)] + 1
                if (nx, ny) not in cost or new_cost < cost[(nx,ny)]:
                    cost[(nx, ny)] = new_cost
                    pr = new_cost + abs(nx - gx) + abs(ny - gy)
                    heapq.heappush(pq, (pr, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)

        if (gx, gy) not in came_from:
            self.path = []
            return []

        # reconstruct
        path = [(gx, gy)]
        while path[-1] != (sx, sy):
            path.append(came_from[path[-1]])
        path.reverse()

        # convert to world
        self.path = [env.grid_to_world(px, py) for px, py in path]
        self.current_wp = 0
        return self.path

    def compute_force(self, pos, goal, env, others=None):

        if not self.path:
            self.plan(pos, goal, env)

        if not self.path:
            return np.zeros(2), {}

        wp = np.array(self.path[self.current_wp])
        diff = wp - pos
        dist = np.linalg.norm(diff)

        if dist < 0.5 and self.current_wp < len(self.path)-1:
            self.current_wp += 1

        if dist < 1e-6:
            return np.zeros(2), {}

        return (diff / dist * 4.0), {}
