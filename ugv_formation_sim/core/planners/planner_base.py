class PathPlanner:
    def plan(self, start, goal, obstacles):
        """
        Args:
            start (tuple): (x, y)
            goal (tuple): (x, y)
            obstacles (list): [(x_center, y_center, radius), ...]

        Returns:
            path: list of (x, y) waypoints
        """
        raise NotImplementedError
