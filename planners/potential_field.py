import numpy as np


class PotentialFieldPlanner:
    """
    Potential Field navigation for a single UAV.

    Computes:
    - Attractive force toward goal
    - Repulsive force from obstacles
    - Repulsive force from other UAVs
    """

    def __init__(self,
                 k_att=1.2,
                 k_rep=2.5,
                 k_rep_uav=1.0,
                 obstacle_influence=3.0,
                 uav_influence=4.0,
                 force_limit=5.0):
        """
        Parameters:
        k_att: attractive gain
        k_rep: repulsive gain for static obstacles
        k_rep_uav: repulsive gain for other UAVs
        obstacle_influence: influence radius for obstacles (meters)
        uav_influence: influence radius for other UAVs
        force_limit: max magnitude of final force
        """

        self.k_att = k_att
        self.k_rep = k_rep
        self.k_rep_uav = k_rep_uav

        self.obstacle_influence = obstacle_influence
        self.uav_influence = uav_influence

        self.force_limit = force_limit

    # Attractive Force
    def attractive_force(self, pos, goal):
        """
        Attractive force toward the goal.
        """
        direction = goal - pos
        dist = np.linalg.norm(direction)

        if dist < 1e-6:
            return np.zeros(2)

        force = self.k_att * (direction / dist)
        return force

    # Repulsive Force from Obstacles
    def obstacle_repulsive_force(self, pos, env):
        """
        Compute repulsive force from nearby obstacles.
        env: OccupancyGrid
        """

        gx, gy = env.world_to_grid(pos[0], pos[1])
        total_force = np.zeros(2)

        rad_cells = int(self.obstacle_influence / env.resolution)

        # Loop over a local window
        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                nx = gx + dx
                ny = gy + dy

                if not env.inside_grid(nx, ny):
                    continue

                if env.get_occupancy_grid(nx, ny) == 1:

                    # Convert grid to world position
                    ox, oy = env.grid_to_world(nx, ny)
                    diff = pos - np.array([ox, oy])
                    dist = np.linalg.norm(diff)

                    if dist < 1e-5:
                        continue

                    if dist < self.obstacle_influence:
                        rep_strength = self.k_rep * (1.0 / dist - 1.0 / self.obstacle_influence)
                        rep_strength = max(rep_strength, 0)

                        total_force += rep_strength * (diff / dist)

        return total_force

    # Repulsive Force from Other UAVs
    def uav_repulsive_force(self, pos, other_positions):
        """
        Repulsive force from other UAVs.
        """
        total_force = np.zeros(2)

        for other in other_positions:
            diff = pos - other
            dist = np.linalg.norm(diff)

            if dist < 1e-5:
                continue

            if dist < self.uav_influence:
                rep_strength = self.k_rep_uav * (1.0 / dist - 1.0 / self.uav_influence)
                rep_strength = max(rep_strength, 0)

                total_force += rep_strength * (diff / dist)

        return total_force

    # Combined Potential Field
    def compute_force(self, pos, goal, env, other_uavs=None):
        """
        Computes the final vector force at the given position.
        """

        other_uavs = other_uavs if other_uavs is not None else []

        f_att = self.attractive_force(pos, goal)
        f_obs = self.obstacle_repulsive_force(pos, env)
        f_uav = self.uav_repulsive_force(pos, other_uavs)

        force = f_att + f_obs + f_uav

        # Limit magnitude
        mag = np.linalg.norm(force)
        if mag > self.force_limit:
            force = (force / mag) * self.force_limit

        return force, {
            "attractive": f_att,
            "obstacles": f_obs,
            "uavs": f_uav,
            "total": force
        }
