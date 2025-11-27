import math

class UGV:
    def __init__(self, x, y, theta, wheelbase=0.5):
        """
        UGV using Ackermann steering model. may change depeding on ilyas's inputs
        Args:
            x (float): initial X position
            y (float): initial Y position
            theta (float): initial orientation (rad)
            wheelbase (float): distance between front & rear axle
        """
        self.x = x
        self.y = y
        self.theta = theta  # radians

        # Control inputs
        self.v = 0.0       # m/s
        self.delta = 0.0   # rad (steering angle)

        # Physical limits
        self.max_speed = 2.0          # m/s
        self.max_steering = math.radians(20)  # 35° steering limit
        self.acc_limit = 1.0          # m/s² acceleration limit

        self.L = wheelbase

    # Set control inputse
    
    def set_control(self, v_cmd, delta_cmd):
        """
        Apply desired control inputs with saturation.
        """
        # Clip linear velocity
        self.v = max(-self.max_speed, min(self.max_speed, v_cmd))

        # Clip steering angle
        self.delta = max(-self.max_steering,
                         min(self.max_steering, delta_cmd))
        
    # Robot kinematic update — called at 50 Hz (dt = 0.02)

    def update(self, dt):
        """
        Update robot pose using Ackermann model.
        
        x_dot = v * cos(theta)
        y_dot = v * sin(theta)
        theta_dot = (v / L) * tan(delta)
        """
        # Kinematics
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        theta_dot = (self.v / self.L) * math.tan(self.delta)

        # Integrate
        self.x += x_dot * dt
        self.y += y_dot * dt
        self.theta += theta_dot * dt

        # Normalize heading to [-π, π]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    # Helper: return current pose
    def pose(self):
        return (self.x, self.y, self.theta)

    # Reset function if needed
    def reset(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.delta = 0.0
        self.L = wheelbase

    def update(self, dt):
        """Update UGV state using Ackermann kinematics."""
        pass

"""
NOTES:
Velocity limited to ±2 m/s

Steering limited to ±35°

Prevents unrealistic robot behavior

Clean Pose Interface

Normalizes θ

Keeps angle in [−π,π].
"""