import numpy as np


class MultiUAVSimulation:
    """
    Core simulation loop for 2 UAVs using:
    - Potential Field planner
    - Bicycle model controller
    - Occupancy grid environment
    """

    def __init__(self, env, planner, controller,
                 goal1, goal2,
                 dt=0.1,
                 safety_distance=1.0):
        """
        env: OccupancyGrid
        planner: PotentialFieldPlanner
        controller: UAVController (shared or per-UAV)
        goal1, goal2: numpy arrays (x,y)
        """

        self.env = env
        self.planner = planner
        self.controller = controller  # same controller object for both UAVs
        self.dt = dt

        # Goals
        self.goal1 = np.array(goal1, dtype=float)
        self.goal2 = np.array(goal2, dtype=float)

        # Collision avoidance radius between UAVs
        self.safety_distance = safety_distance

        # UAV states: x, y, theta, v, phi
        self.uav1_state = np.array([2.5, 2.5, np.pi / 4, 0.0, 0.0], dtype=float)
        self.uav2_state = np.array([2.5, 27.5, -np.pi / 4, 0.0, 0.0], dtype=float)

        # Status flags
        self.goal1_reached = False
        self.goal2_reached = False
        self.running = False

    # Simulation control
    def start(self):
        self.running = True
        self.goal1_reached = False
        self.goal2_reached = False

    def stop(self):
        self.running = False

    def reset(self):
        """Reset UAV states & flags."""
        self.uav1_state[:] = [2.5, 2.5, np.pi/4, 0.0, 0.0]
        self.uav2_state[:] = [2.5, 27.5, -np.pi/4, 0.0, 0.0]
        self.goal1_reached = False
        self.goal2_reached = False

    # Helpers
    def check_goals(self):
        """Check if UAVs reached their goals."""
        if not self.goal1_reached:
            if np.linalg.norm(self.uav1_state[:2] - self.goal1) < 0.5:
                self.goal1_reached = True

        if not self.goal2_reached:
            if np.linalg.norm(self.uav2_state[:2] - self.goal2) < 0.5:
                self.goal2_reached = True

    def check_uav_collision(self):
        """Stop UAV2 if too close to UAV1 (priority behavior)."""
        p1 = self.uav1_state[:2]
        p2 = self.uav2_state[:2]
        d = np.linalg.norm(p1 - p2)
        return d < self.safety_distance

    # Main simulation tick
    def step(self):
        if not self.running:
            return

        # 1. Check for mutual collision
        uav2_can_move = True

        if self.check_uav_collision():
            # UAV1 has priority → freeze UAV2
            uav2_can_move = False

        # 2. Compute forces for UAV1
        if not self.goal1_reached:
            # Other UAV position used for repulsion
            other1 = [self.uav2_state[:2]]
            force1, dbg1 = self.planner.compute_force(
                self.uav1_state[:2], self.goal1, self.env, other1
            )
            # Apply controller
            self.uav1_state = self.controller.step(self.uav1_state, force1)

        # 3. Compute forces for UAV2
        if not self.goal2_reached and uav2_can_move:
            other2 = [self.uav1_state[:2]]
            force2, dbg2 = self.planner.compute_force(
                self.uav2_state[:2], self.goal2, self.env, other2
            )
            self.uav2_state = self.controller.step(self.uav2_state, force2)

        # 4. Check goals
        self.check_goals()

        # 5. Stop if both reached
        if self.goal1_reached and self.goal2_reached:
            self.running = False

    # Data for GUI drawing
    def get_states(self):
        """Return copies of UAV states for drawing."""
        return (
            np.copy(self.uav1_state),
            np.copy(self.uav2_state),
            np.copy(self.goal1),
            np.copy(self.goal2),
        )
