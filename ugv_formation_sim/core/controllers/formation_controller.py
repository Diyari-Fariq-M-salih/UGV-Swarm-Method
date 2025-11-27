class FormationController:
    def __init__(self, formation_type="triangle"):
        self.formation_type = formation_type

    def get_offsets(self):
        """Return δ_i offsets for the current formation."""
        pass

    def assign_targets(self, leader_pose):
        """
        Computes desired targets for each robot.
        """
        pass
