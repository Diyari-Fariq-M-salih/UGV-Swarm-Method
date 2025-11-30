import numpy as np
import random

class RRTPlanner:

    def __init__(self, step=1.2, max_iter=2000):
        self.step = step
        self.max_iter = max_iter
        self.path = []
        self.current_wp = 0

    def collision(self, x, y, env):
        gx, gy = env.world_to_grid(x, y)
        return env.get_occupancy_grid(gx, gy) == 1

    def sample(self, env):
        return (
            random.uniform(0, env.width),
            random.uniform(0, env.height)
        )

    def nearest(self, nodes, x, y):
        return min(nodes, key=lambda n: (n[0]-x)**2 + (n[1]-y)**2)

    def steer(self, from_node, to_point):
        fx, fy = from_node
        tx, ty = to_point

        v = np.array([tx - fx, ty - fy])
        dist = np.linalg.norm(v)
        if dist < self.step:
            return (tx, ty)

        v = v / dist * self.step
        return (fx + v[0], fy + v[1])

    def plan(self, start, goal, env):
        nodes = [tuple(start)]
        parents = {tuple(start): None}

        for _ in range(self.max_iter):
            rnd = goal if random.random() < 0.1 else self.sample(env)
            nearest_node = self.nearest(nodes, *rnd)
            new_node = self.steer(nearest_node, rnd)

            if self.collision(*new_node, env):
                continue

            nodes.append(new_node)
            parents[new_node] = nearest_node

            if np.linalg.norm(np.array(new_node) - np.array(goal)) < 1.5:
                # -------- FIX: Safe reconstruction 
                path = []
                node = new_node
                while node is not None:
                    path.append(node)
                    node = parents[node]
                path.reverse()
                # -------- FIX: Safe reconstruction 

                self.path = path
                self.current_wp = 0
                return path

        self.path = []
        return []

    def compute_force(self, pos, goal, env, others=None):

        if not self.path:
            self.plan(pos, goal, env)

        if not self.path:
            return np.zeros(2), {}

        wp = np.array(self.path[self.current_wp])
        diff = wp - pos
        dist = np.linalg.norm(diff)

        if dist < 0.5 and self.current_wp < len(self.path) - 1:
            self.current_wp += 1

        if dist < 1e-6:
            return np.zeros(2), {}

        return (diff / dist * 4.0), {}
