import random
import math
from .planner_base import PathPlanner
from .collision import collision_point, collision_segment

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRTPlanner(PathPlanner):
    def __init__(self, step_size=0.5, max_iter=2000, goal_bias=0.05):
        self.step = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias

    def nearest(self, nodes, rnd):
        return min(nodes, key=lambda n: (n.x - rnd[0])**2 + (n.y - rnd[1])**2)

    def steer(self, from_node, to_point):
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        new_x = from_node.x + self.step * math.cos(theta)
        new_y = from_node.y + self.step * math.sin(theta)
        return Node(new_x, new_y, parent=from_node)

    def plan(self, start, goal, obstacles):
        start_node = Node(*start)
        goal_node = Node(*goal)
        nodes = [start_node]

        for _ in range(self.max_iter):

            # Goal biasing
            if random.random() < self.goal_bias:
                rnd = (goal_node.x, goal_node.y)
            else:
                rnd = (random.uniform(-5, 15), random.uniform(-5, 15))

            nearest_node = self.nearest(nodes, rnd)
            new_node = self.steer(nearest_node, rnd)

            # Collision check for point and path edge
            if collision_point((new_node.x, new_node.y), obstacles):
                continue
            if collision_segment((nearest_node.x, nearest_node.y),
                                 (new_node.x, new_node.y),
                                 obstacles):
                continue

            nodes.append(new_node)

            # Check if goal has been reached
            if math.hypot(new_node.x - goal_node.x, new_node.y - goal_node.y) < 1.0:
                return self.extract_path(new_node)

        return []

    def extract_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
