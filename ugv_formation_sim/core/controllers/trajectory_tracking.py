class PurePursuitController:
    def __init__(self, lookahead=1.0, wheelbase=0.5):
        self.lookahead = lookahead
        self.L = wheelbase

    def compute_control(self, robot_state, path):
        """
        Returns:
            v (float): linear velocity
            delta (float): steering angle
        """
        pass
