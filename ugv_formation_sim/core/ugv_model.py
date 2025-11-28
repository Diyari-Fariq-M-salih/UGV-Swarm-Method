import math

"""
UGV using Ackermann steering model. may change depeding on ilyas's inputs
Args:
x (float): initial X position
y (float): initial Y position
theta (float): initial orientation (rad)
wheelbase (float): distance between front & rear axle
"""

class UGV:
    def __init__(self, x, y, theta, wheelbase=0.5):
        self.x = x
        self.y = y
        self.theta = theta  # radians

        self.v = 0.0
        self.delta = 0.0

        self.max_speed = 1.0
        self.max_steering = math.radians(20)
        self.acc_limit = 0.5

        self.L = wheelbase

    def set_control(self, v_cmd, delta_cmd):
        self.v = max(-self.max_speed, min(self.max_speed, v_cmd))
        self.delta = max(-self.max_steering, min(self.max_steering, delta_cmd))

    def update(self, dt):
        x_dot = self.v * math.cos(self.theta)
        y_dot = self.v * math.sin(self.theta)
        theta_dot = (self.v / self.L) * math.tan(self.delta)

        self.x += x_dot * dt
        self.y += y_dot * dt
        self.theta += theta_dot * dt

        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def pose(self):
        return (self.x, self.y, self.theta)

    def reset(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.delta = 0.0

"""
NOTES:
Velocity limited to ±2 m/s

Steering limited to ±35°

Prevents unrealistic robot behavior

Clean Pose Interface

Normalizes θ

Keeps angle in [−π,π].
"""