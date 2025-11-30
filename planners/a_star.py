import heapq
import numpy as np


class AStarPlanner:
    """
    Grid-based A* planner.
    Produces a waypoint path, then converts it to a force vector.
    """

    def __init__(self):
        self.path = []
        self.current_wp = 0

    # A* helpers

    def neighbors(self, gx, gy, env):
        """4-connected grid neighbors."""
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy
            if env.get_occupancy_grid(nx, ny) == 0:
                yield nx, ny

    def plan(self, start, goal, env):
        """Run A* from start to goal."""
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
                if (nx, ny) not in cost or new_cost < cost[(nx, ny)]:
                    cost[(nx, ny)] = new_cost
                    priority = new_cost + abs(nx - gx) + abs(ny - gy)
                    heapq.heappush(pq, (priority, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)

        # Cannot reach goal
        if (gx, gy) not in came_from:
            self.path = []
            return []

        # Reconstruct reversed path
        path = [(gx, gy)]
        while path[-1] != (sx, sy):
            path.append(came_from[path[-1]])

        path.reverse()

        # Convert to world coords
        self.path = [env.grid_to_world(px, py) for px, py in path]
        self.current_wp = 0
        return self.path

    # Path following → force vector

    def compute_force(self, pos, goal, env, others=None):
        if not self.path:
            self.plan(pos, goal, env)

        if not self.path:
            return np.zeros(2), {}

        wp = np.array(self.path[self.current_wp])
        diff = wp - pos
        dist = np.linalg.norm(diff)

        # Advance waypoint
        if dist < 0.5 and self.current_wp < len(self.path) - 1:
            self.current_wp += 1

        if dist < 1e-6:
            return np.zeros(2), {}

        # Convert heading to force like PF planner
        return (diff / dist * 4.0), {}
