import numpy as np

class PotentialFieldPlanner:
    """
    Attractive + repulsive potential field planner.
    """

    def __init__(self,
                 k_att=1.2,
                 k_rep=2.5,
                 k_rep_uav=1.0,
                 obstacle_influence=3.0,
                 uav_influence=3.0,
                 force_limit=4.0):

        self.k_att = k_att
        self.k_rep = k_rep
        self.k_rep_uav = k_rep_uav
        self.obstacle_influence = obstacle_influence
        self.uav_influence = uav_influence
        self.force_limit = force_limit

    def attractive_force(self, pos, goal):
        d = goal - pos
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            return np.zeros(2)
        return self.k_att * (d / dist)

    def obstacle_repulsive_force(self, pos, env):
        gx, gy = env.world_to_grid(pos[0], pos[1])
        total = np.zeros(2)
        rad_cells = int(self.obstacle_influence / env.resolution)

        for dy in range(-rad_cells, rad_cells+1):
            for dx in range(-rad_cells, rad_cells+1):
                nx, ny = gx + dx, gy + dy
                if env.get_occupancy_grid(nx, ny) != 1:
                    continue

                ox, oy = env.grid_to_world(nx, ny)
                diff = pos - np.array([ox, oy])
                dist = np.linalg.norm(diff)
                if 0 < dist < self.obstacle_influence:
                    rep = self.k_rep * (1.0/dist - 1.0/self.obstacle_influence)
                    total += rep * (diff / dist)

        return total

    def uav_repulsive_force(self, pos, others):
        total = np.zeros(2)
        for o in others:
            diff = pos - o
            dist = np.linalg.norm(diff)
            if 0 < dist < self.uav_influence:
                rep = self.k_rep_uav * (1.0/dist - 1.0/self.uav_influence)
                total += rep * (diff / dist)
        return total

    def compute_force(self, pos, goal, env, other_uavs=None):
        if other_uavs is None:
            other_uavs = []

        f_att = self.attractive_force(pos, goal)
        f_obs = self.obstacle_repulsive_force(pos, env)
        f_uav = self.uav_repulsive_force(pos, other_uavs)

        f = f_att + f_obs + f_uav

        mag = np.linalg.norm(f)
        if mag > self.force_limit:
            f = f / mag * self.force_limit

        return f, {
            "attractive": f_att,
            "obstacles": f_obs,
            "uavs": f_uav,
            "total": f
        }
