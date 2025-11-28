import numpy as np


class UAVController:
    """
    Converts potential field force vector into steering and velocity commands,
    then applies a bicycle model for state propagation.
    """

    def __init__(self,
                 wheelbase=0.5,
                 dt=0.1,
                 max_vel=2.0,
                 max_accel=1.0,
                 max_steer=np.pi/6,
                 max_steer_rate=np.pi/4):
        """
        Parameters match the MATLAB reference.
        """

        self.L = wheelbase
        self.dt = dt

        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate

    # Controller: PF force → control commands
    def compute_control(self, state, force):
        """
        Convert PF force vector into:
        - desired heading
        - desired steering
        - desired velocity
        """

        x, y, theta, v, phi = state
        fx, fy = force

        # Desired heading is direction of the force vector
        desired_heading = np.arctan2(fy, fx)

        # Heading error normalized
        heading_error = np.arctan2(
            np.sin(desired_heading - theta),
            np.cos(desired_heading - theta)
        )

        # Target steering angle via simple proportional method
        # phi_target = steering angle that turns toward desired heading
        # bicycle kinematics: theta_dot = (v / L) * tan(phi)
        # → tan(phi_target) proportional to heading error
        phi_target = np.arctan2(2 * self.L * np.sin(heading_error), v + 0.1)

        # Saturate
        phi_target = np.clip(phi_target, -self.max_steer, self.max_steer)

        # Steering rate limit
        phi_error = phi_target - phi
        dphi = np.clip(phi_error * 3.0,   # steering P-gain
                       -self.max_steer_rate,
                        self.max_steer_rate)

        # Velocity target proportional to force magnitude
        desired_speed = min(np.linalg.norm(force), self.max_vel)

        # Acceleration limit
        dv = np.clip(desired_speed - v,
                     -self.max_accel,
                      self.max_accel)

        return dv, dphi

    # Bicycle Model Integration
    def step(self, state, force):
        """
        Perform one simulation integration step.
        state = [x, y, theta, v, phi]
        """

        x, y, theta, v, phi = state

        # Compute DV and DPHI commands
        dv, dphi = self.compute_control(state, force)

        # Update velocity and steering
        v_new = v + dv * self.dt
        v_new = np.clip(v_new, 0, self.max_vel)

        phi_new = phi + dphi * self.dt
        phi_new = np.clip(phi_new, -self.max_steer, self.max_steer)

        # Bicycle kinematics
        x_new = x + v_new * np.cos(theta) * self.dt
        y_new = y + v_new * np.sin(theta) * self.dt

        theta_new = theta + (v_new / self.L) * np.tan(phi_new) * self.dt
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))  # wrap angle

        return np.array([x_new, y_new, theta_new, v_new, phi_new])
