import random
import math
from .planner_base import PathPlanner

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTPlanner(PathPlanner):
    def __init__(self, step_size=0.5, max_iter=2000, goal_bias=0.05):
        self.step = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias

    def collision(self, x, y, obstacles):
        for ox, oy, rad in obstacles:
            if math.hypot(x-ox, y-oy) <= rad:
                return True
        return False

    def nearest(self, nodes, rnd):
        return min(nodes, key=lambda n: (n.x-rnd[0])**2 + (n.y-rnd[1])**2)

    def steer(self, from_node, to_point):
        theta = math.atan2(to_point[1]-from_node.y, to_point[0]-from_node.x)
        new_x = from_node.x + self.step * math.cos(theta)
        new_y = from_node.y + self.step * math.sin(theta)
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def plan(self, start, goal, obstacles):
        start_node = Node(*start)
        goal_node = Node(*goal)
        nodes = [start_node]

        for _ in range(self.max_iter):

            if random.random() < self.goal_bias:
                rnd = (goal_node.x, goal_node.y)
            else:
                rnd = (random.uniform(-5, 15), random.uniform(-5, 15))

            nearest_node = self.nearest(nodes, rnd)
            new_node = self.steer(nearest_node, rnd)

            if self.collision(new_node.x, new_node.y, obstacles):
                continue

            nodes.append(new_node)

            if math.hypot(new_node.x - goal_node.x, new_node.y - goal_node.y) < 1.0:
                return self.get_path(new_node)

        return []

    def get_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
