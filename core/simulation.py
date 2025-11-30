import numpy as np
import copy

class DynamicObstacle:
    def __init__(self, x, y, vx, vy, size=1.0):
        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([vx, vy], dtype=float)
        self.size = size

    def step(self, dt):
        self.pos += self.vel * dt


class MultiUAVSimulation:

    def __init__(self, env, planner, controller,
                 goal1, goal2, dt=0.1, safety_distance=5.0):

        self.env = env

        self.planner1 = copy.deepcopy(planner)
        self.planner2 = copy.deepcopy(planner)

        self.controller = controller
        self.dt = dt
        self.safety_distance = 5.0   # gentle avoidance radius

        self.goal1 = np.array(goal1)
        self.goal2 = np.array(goal2)

        self.uav1_state = np.array([2.5, 2.5, np.pi/4, 0.0, 0.0])
        self.uav2_state = np.array([2.5, 27.5, -np.pi/4, 0.0, 0.0])

        self.goal1_reached = False
        self.goal2_reached = False
        self.running = False

    def compute_paths(self):
        self.planner1.plan(self.uav1_state[:2], self.goal1, self.env)
        self.planner2.plan(self.uav2_state[:2], self.goal2, self.env)

    # ------------- PURE PURSUIT FOLLOWER --------------
    def pure_pursuit_force(self, pos, planner, lookahead=1.5):
        if not planner.path:
            return np.zeros(2)

        # Find lookahead point
        for i in range(planner.current_wp, len(planner.path)):
            wp = np.array(planner.path[i])
            if np.linalg.norm(wp - pos) > lookahead:
                planner.current_wp = i
                break

        wp = np.array(planner.path[planner.current_wp])
        diff = wp - pos
        dist = np.linalg.norm(diff)

        if dist < 0.5 and planner.current_wp < len(planner.path)-1:
            planner.current_wp += 1

        if dist < 1e-6:
            return np.zeros(2)

        return diff / dist

    # ------------- GENTLE AVOIDANCE --------------------
    def avoidance_force(self, pos, other_pos, radius=5.0, k=0.5):
        diff = pos - other_pos
        dist = np.linalg.norm(diff)
        if dist >= radius or dist < 1e-6:
            return np.zeros(2)

        return k * (1/dist - 1/radius) * (diff / dist)

    # ------------- SIMULATION STEP ---------------------
    def step(self):
        if not self.running:
            return

        # UAV1
        f1_path = self.pure_pursuit_force(self.uav1_state[:2], self.planner1)
        f1_avoid = self.avoidance_force(self.uav1_state[:2], self.uav2_state[:2])
        f1 = f1_path + f1_avoid
        self.uav1_state = self.controller.step(self.uav1_state, f1)

        # UAV2
        f2_path = self.pure_pursuit_force(self.uav2_state[:2], self.planner2)
        f2_avoid = self.avoidance_force(self.uav2_state[:2], self.uav1_state[:2])
        f2 = f2_path + f2_avoid
        self.uav2_state = self.controller.step(self.uav2_state, f2)

        # Goal checks
        if np.linalg.norm(self.uav1_state[:2] - self.goal1) < 0.5:
            self.goal1_reached = True
        if np.linalg.norm(self.uav2_state[:2] - self.goal2) < 0.5:
            self.goal2_reached = True

        if self.goal1_reached and self.goal2_reached:
            self.running = False

    def start(self):
        self.running = True
        self.goal1_reached = False
        self.goal2_reached = False

    def stop(self):
        self.running = False

    def reset(self):
        self.uav1_state = np.array([2.5, 2.5, np.pi/4, 0.0, 0.0])
        self.uav2_state = np.array([2.5, 27.5, -np.pi/4, 0.0, 0.0])

        self.planner1.path = []
        self.planner2.path = []
        self.planner1.current_wp = 0
        self.planner2.current_wp = 0

        self.goal1_reached = False
        self.goal2_reached = False
        self.running = False

    def get_states(self):
        return (
            np.copy(self.uav1_state),
            np.copy(self.uav2_state),
            np.copy(self.goal1),
            np.copy(self.goal2),
            []
        )
