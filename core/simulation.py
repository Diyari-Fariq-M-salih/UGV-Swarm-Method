import numpy as np

class DynamicObstacle:
    """
    Simple moving obstacle.
    pos = world position
    vel = (vx, vy)
    size = square size in meters
    """
    def __init__(self, x, y, vx, vy, size=1.0):
        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([vx, vy], dtype=float)
        self.size = size

    def step(self, dt):
        self.pos += self.vel * dt


class MultiUAVSimulation:
    """
    2-UAV PF-based simulator with moving dynamic obstacles.
    """

    def __init__(self, env, planner, controller,
                 goal1, goal2, dt=0.1, safety_distance=2.0):

        self.env = env
        self.planner = planner
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

    # Dynamic Obstacle API
    def add_horizontal_obstacle(self, y, speed=1.0):
        self.dynamic_obstacles.append(DynamicObstacle(
            x=2.0, y=y, vx=speed, vy=0.0, size=0.5))

    def add_vertical_obstacle(self, x, speed=1.0):
        self.dynamic_obstacles.append(DynamicObstacle(
            x=x, y=2.0, vx=0.0, vy=speed, size=0.5))

    def start(self):
        self.running = True
        self.goal1_reached = False
        self.goal2_reached = False

    def stop(self):
        self.running = False

    def reset(self):
        self.uav1_state[:] = [2.5, 2.5, np.pi/4, 0.0, 0.0]
        self.uav2_state[:] = [2.5, 27.5, -np.pi/4, 0.0, 0.0]
        self.goal1_reached = False
        self.goal2_reached = False
        # clear dynamic obstacles from map
        self.dynamic_obstacles = []

    def check_goals(self):
        if not self.goal1_reached:
            if np.linalg.norm(self.uav1_state[:2] - self.goal1) < 0.5:
                self.goal1_reached = True
        if not self.goal2_reached:
            if np.linalg.norm(self.uav2_state[:2] - self.goal2) < 0.5:
                self.goal2_reached = True

    def check_uav_collision(self):
        p1, p2 = self.uav1_state[:2], self.uav2_state[:2]
        return np.linalg.norm(p1 - p2) < self.safety_distance

    def step(self):
        if not self.running:
            return

        # Update dynamic obstacles
        self.env.clear()

        for obs in self.dynamic_obstacles:
            obs.step(self.dt)

            # bounce on borders
            if obs.pos[0] <= 1 or obs.pos[0] >= self.env.width - 1:
                obs.vel[0] *= -1
            if obs.pos[1] <= 1 or obs.pos[1] >= self.env.height - 1:
                obs.vel[1] *= -1

            self.env.paint_square(obs.pos[0], obs.pos[1], obs.size * 2)

        # Collision between UAVs
        uav2_can_move = not self.check_uav_collision()

        # UAV1 control
        if not self.goal1_reached:
            force1, _ = self.planner.compute_force(
                self.uav1_state[:2], self.goal1, self.env,
                [self.uav2_state[:2]]
            )
            self.uav1_state = self.controller.step(self.uav1_state, force1)

        # UAV2 control
        if not self.goal2_reached and uav2_can_move:
            force2, _ = self.planner.compute_force(
                self.uav2_state[:2], self.goal2, self.env,
                [self.uav1_state[:2]]
            )
            self.uav2_state = self.controller.step(self.uav2_state, force2)

        self.check_goals()
        if self.goal1_reached and self.goal2_reached:
            self.running = False

    def get_states(self):
        return (
            np.copy(self.uav1_state),
            np.copy(self.uav2_state),
            np.copy(self.goal1),
            np.copy(self.goal2),
            self.dynamic_obstacles
        )
