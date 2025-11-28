import numpy as np

class UAVController:
    """
    PF → steering & velocity → bicycle kinematics.
    dt affects physics frequency, NOT rendering FPS.
    """

    def __init__(self,
                 wheelbase=0.5,
                 dt=0.1,               # physics step (10 Hz)
                 max_vel=2.0,
                 max_accel=1.0,
                 max_steer=np.pi/6,
                 max_steer_rate=np.pi/4):

        self.L = wheelbase
        self.dt = dt
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate

    def compute_control(self, state, force):
        x, y, theta, v, phi = state
        fx, fy = force

        desired_heading = np.arctan2(fy, fx)
        heading_error = np.arctan2(
            np.sin(desired_heading - theta),
            np.cos(desired_heading - theta)
        )

        phi_target = np.arctan2(
            2 * self.L * np.sin(heading_error),
            v + 0.1
        )
        phi_target = np.clip(phi_target, -self.max_steer, self.max_steer)

        phi_error = phi_target - phi
        dphi = np.clip(phi_error * 3.0,
                       -self.max_steer_rate,
                        self.max_steer_rate)

        desired_speed = min(np.linalg.norm(force), self.max_vel)
        dv = np.clip(desired_speed - v,
                     -self.max_accel,
                      self.max_accel)

        return dv, dphi

    def step(self, state, force):
        x, y, theta, v, phi = state
        dv, dphi = self.compute_control(state, force)

        v_new = np.clip(v + dv * self.dt, 0, self.max_vel)
        phi_new = np.clip(phi + dphi * self.dt,
                          -self.max_steer, self.max_steer)

        x_new = x + v_new * np.cos(theta) * self.dt
        y_new = y + v_new * np.sin(theta) * self.dt

        theta_new = theta + (v_new / self.L) * np.tan(phi_new) * self.dt
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        return np.array([x_new, y_new, theta_new, v_new, phi_new])
