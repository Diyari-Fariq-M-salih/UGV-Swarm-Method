import numpy as np
import random
import time

class RRTPlanner:

    def __init__(self, step=1.2, max_iter=2000):
        self.step = step
        self.max_iter = max_iter
        self.path = []
        self.current_wp = 0

        self.tree_edges = []
        self.plan_time = 0.0
        self.node_count = 0
        self.path_length = 0.0

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

    # -------- FIXED & SAFE SMOOTHER --------
    def smooth_path(self, path, env, iterations=80):
        # Must have 4 or more points
        if len(path) < 4:
            return path

        smooth = path.copy()

        for _ in range(iterations):
            # Protect after path shrinks
            if len(smooth) < 4:
                break

            max_i = len(smooth) - 3
            max_j = len(smooth) - 1

            if max_i <= 0:
                break

            i = random.randint(0, max_i)
            j = random.randint(i+2, max_j)

            p1 = np.array(smooth[i])
            p2 = np.array(smooth[j])

            # Check collision along the direct line
            steps = max(2, int(np.linalg.norm(p2 - p1) / env.resolution))
            collision = False

            for k in range(steps):
                t = k / steps
                x = p1[0]*(1-t) + p2[0]*t
                y = p1[1]*(1-t) + p2[1]*t
                if self.collision(x, y, env):
                    collision = True
                    break

            if not collision:
                # Replace section with shortcut
                smooth = smooth[:i+1] + smooth[j:]

        return smooth

    # ---------------------------------------
    #  RRT PLANNING
    # ---------------------------------------
    def plan(self, start, goal, env):
        t0 = time.time()
        nodes = [tuple(start)]
        parents = {tuple(start): None}

        self.tree_edges = []
        self.node_count = 1

        for _ in range(self.max_iter):

            rnd = goal if random.random() < 0.1 else self.sample(env)
            nearest_node = self.nearest(nodes, *rnd)
            new_node = self.steer(nearest_node, rnd)

            if self.collision(*new_node, env):
                continue

            nodes.append(new_node)
            parents[new_node] = nearest_node
            self.tree_edges.append((nearest_node, new_node))
            self.node_count += 1

            # Goal reached
            if np.linalg.norm(np.array(new_node) - np.array(goal)) < 1.5:

                # Reconstruct path
                path = []
                node = new_node
                while node is not None:
                    path.append(node)
                    node = parents[node]
                path.reverse()

                # --- SAFE SMOOTHING ---
                smooth = self.smooth_path(path, env, iterations=2)

                self.path = smooth
                self.current_wp = 0

                self.plan_time = (time.time() - t0) * 1000.0
                self.path_length = sum(
                    np.linalg.norm(np.array(smooth[i]) - np.array(smooth[i+1]))
                    for i in range(len(smooth)-1)
                )

                return smooth

        self.path = []
        return []

    def compute_force(self, pos, goal, env, others=None):
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
