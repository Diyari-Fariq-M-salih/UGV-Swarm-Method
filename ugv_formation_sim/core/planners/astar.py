import heapq
import math
from .planner_base import PathPlanner
from .collision import collision_point

class AStarPlanner(PathPlanner):
    def __init__(self, grid_resolution=0.5):
        self.res = grid_resolution

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, node):
        x, y = node
        r = self.res
        directions = [
            (r, 0), (-r, 0), (0, r), (0, -r),
            (r, r), (r, -r), (-r, r), (-r, -r),
        ]
        return [(x + dx, y + dy) for dx, dy in directions]

    def snap(self, x, y):
        return (
            round(x / self.res) * self.res,
            round(y / self.res) * self.res
        )

    def plan(self, start, goal, obstacles):
        start = self.snap(*start)
        goal  = self.snap(*goal)

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if self.heuristic(current, goal) < self.res:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if collision_point(neighbor, obstacles):
                    continue

                tentative_g = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current

        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
