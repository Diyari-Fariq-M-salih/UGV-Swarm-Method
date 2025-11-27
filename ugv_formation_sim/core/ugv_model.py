class UGV:
    def __init__(self, x, y, theta, wheelbase=0.5):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.delta = 0.0
        self.L = wheelbase

    def update(self, dt):
        """Update UGV state using Ackermann kinematics."""
        pass
