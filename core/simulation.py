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

        # ------------------------------
        # FIX: Split planner
        # ------------------------------
        import copy
        self.planner1 = copy.deepcopy(planner)
        self.planner2 = copy.deepcopy(planner)

        self.controller = controller
        self.dt = dt
        self.safety_distance = safety_distance

        self.goal1 = np.array(goal1)
        self.goal2 = np.array(goal2)

        self.uav1_state = np.array([2.5, 2.5, np.pi/4, 0.0, 0.0])
        self.uav2_state = np.array([2.5, 27.5, -np.pi/4, 0.0, 0.0])

        self.goal1_reached = False
        self.goal2_reached = False
        self.running = False

        self.dynamic_obstacles = []


    def step(self):
        if not self.running:
            return

        self.env.clear_dynamic()

        # UAV1
        force1, _ = self.planner1.compute_force(
            self.uav1_state[:2], self.goal1, self.env,
            [self.uav2_state[:2]]
        )
        self.uav1_state = self.controller.step(self.uav1_state, force1)

        # UAV2
        force2, _ = self.planner2.compute_force(
            self.uav2_state[:2], self.goal2, self.env,
            [self.uav1_state[:2]]
        )
        self.uav2_state = self.controller.step(self.uav2_state, force2)

        # goal check
        if np.linalg.norm(self.uav1_state[:2] - self.goal1) < 0.5:
            self.goal1_reached = True
        if np.linalg.norm(self.uav2_state[:2] - self.goal2) < 0.5:
            self.goal2_reached = True

        if self.goal1_reached and self.goal2_reached:
            self.running = False
            
    def start(self):
        """Start the simulation loop."""
        self.running = True
        self.goal1_reached = False
        self.goal2_reached = False

    def stop(self):
        """Stop the simulation loop."""
        self.running = False

    def reset(self):
        """Reset UAV positions and planner paths (static map unchanged)."""
        # Reset UAVs
        self.uav1_state = np.array([2.5, 2.5, np.pi/4, 0.0, 0.0])
        self.uav2_state = np.array([2.5, 27.5, -np.pi/4, 0.0, 0.0])

        # Reset planners
        self.planner1.path = []
        self.planner1.current_wp = 0
        self.planner2.path = []
        self.planner2.current_wp = 0

        # Reset flags
        self.goal1_reached = False
        self.goal2_reached = False

        # Do not modify environment or dynamic obstacles
        self.running = False

    def get_states(self):
        return (
            np.copy(self.uav1_state),
            np.copy(self.uav2_state),
            np.copy(self.goal1),
            np.copy(self.goal2),
            self.dynamic_obstacles
        )
