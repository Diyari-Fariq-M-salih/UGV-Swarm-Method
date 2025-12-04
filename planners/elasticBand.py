import numpy as np

class Bubble:
    """
    A bubble is approximated by a circle around a configuration point with a certain radius.
    TODO: implement anisotropic bubble with different radii for x, y, theta with anisotropic potential field.
    """
    def __init__(self, configuration, radius):
        # configuration of the robot: (x, y, theta) 
        self.x = configuration[0]
        self.y = configuration[1]
        self.theta = configuration[2] if len(configuration) > 2 else 0.0  # Add default theta
        # considering single radius for simplicity
        # TODO: implement anisotropic bubble with different radii for x, y, theta
        self.radius = radius

class elasticBand:
    def __init__(self, global_path, env, **params):
        
        # Get eband_params and set defaults
        eband_params = params.get('eband_params', {})
        self.kc = eband_params.get('kc', 1.0)  # contraction constant
        self.kr = eband_params.get('kr', 2.0)  # repulsion constant
        self.k = eband_params.get('k_potential_field', 1.5)  # potential field weight constant
        self.alpha = eband_params.get('alpha', 0.1)  # step size for updates
        self.d_inflation = eband_params.get('inflation_dist', 0.5)  # safety distance (influence radius)
        rho0_tol = eband_params.get('rho0_tol', 0.2)
        self.base_clearance = eband_params.get('base_clearance', 10.0)  # maximum clearance value
        # Get robot_params with defaults
        robot_params = params.get('robot_params', {})
        self.robot_radius = robot_params.get('radius', 0.3)  # robot radius
        
        # Derived parameters
        self.d_safe = self.robot_radius + self.d_inflation  # safe distance from obstacles
        self.rho0 = self.d_safe + rho0_tol  # potential field influence distance typically >= d_safe 
        self.env = env  # store environment reference
        self.init_eband(global_path)

    def init_eband(self, global_path):
        """Initialize the elastic band by creating bubbles around each point."""
        self.bubbles = []
        for point in global_path:
            bubble = Bubble(point, self.robot_radius)
            self.update_bubble_radius(bubble)  # Compute and set initial radius
            self.bubbles.append(bubble)

    # main eband deformation function
    def deform_eband(self):
        """Update the elastic band by applying contraction and repulsion forces."""
        n = len(self.bubbles)
        n_removed = 0
        n_inserted = 0
        # First pass: update positions
        for i in range(1, n - 1):
            b_prev = self.bubbles[i - 1]
            b_curr = self.bubbles[i]
            b_next = self.bubbles[i + 1]

            # Repulsion force from obstacles
            f_repulsion = self.repulsion_force(b_curr)

            # contraction force from neighboring bubbles
            f_contraction = self.contraction_force(b_prev, b_curr, b_next)

            # Total force
            f_total = f_contraction + f_repulsion
            # OPTIONAL:
            # project force to normal direction
            tangent = np.array([b_prev.x - b_next.x, b_prev.y - b_next.y])
            dot_prod = np.dot(f_total, tangent)
            prod = dot_prod * tangent
            norm = np.linalg.norm(np.array(tangent))
            if norm > 1e-6:
                f_total = f_total - prod / (norm**2)

            # Update bubble position
            b_curr.x += self.alpha * f_total[0]
            b_curr.y += self.alpha * f_total[1]


            # Project force perpendicular to band direction (tangent vector)
            tangent = np.array([b_next.x - b_prev.x, b_next.y - b_prev.y])
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 1e-6:
                tangent = tangent / tangent_norm
                # Remove tangent component, keep only perpendicular component
                f_total = f_total - np.dot(f_total, tangent) * tangent

            # Update bubble position
            b_curr.x += self.alpha * f_total[0]
            b_curr.y += self.alpha * f_total[1]

            # Update radius based on new position
            self.update_bubble_radius(b_curr)
        
        # update radius for start and goal points (in case new obstacles appeared)
        self.update_bubble_radius(self.bubbles[0])
        self.update_bubble_radius(self.bubbles[-1])

        # Second pass: Insert bubbles where there are gaps (no overlap)
        i = 0
        while (i < len(self.bubbles) - 1):
            b_curr = self.bubbles[i]
            b_next = self.bubbles[i + 1]
            
            # Check if bubbles DON'T overlap - need to insert
            if not self.check_overlap(b_curr, b_next):
                n_inserted += 1
                self.insert_bubble(i)
                # Don't increment i, check the newly inserted bubble
            else:
                i += 1
        
        # Third pass: Remove redundant bubbles
        # A bubble is redundant if removing it still keeps the band connected
        i = 1
        while (i < len(self.bubbles) - 1):
            b_prev = self.bubbles[i - 1]
            b_next = self.bubbles[i + 1]
            
            # Check if prev and next would still overlap without current bubble
            dist_neighbors = np.linalg.norm(np.array([b_next.x - b_prev.x, b_next.y - b_prev.y]))
            
            # If neighbors would still overlap (with margin), current bubble is redundant
            if (dist_neighbors < (b_prev.radius + b_next.radius)):
                n_removed += 1
                self.remove_bubble(i)
                # Don't increment i, check next bubble at same position
            else:
                i += 1
        
        # if n_removed > 0 or n_inserted > 0:
            # print(f"Removed {n_removed} redundant bubbles. Inserted {n_inserted} new bubbles.")

    # helper functions

    def insert_bubble(self, index):
        """
        Insert a new bubble between bubbles at index and index + 1
        """
        b1 = self.bubbles[index]
        b2 = self.bubbles[index + 1]
        new_x = (b1.x + b2.x) / 2.0
        new_y = (b1.y + b2.y) / 2.0
        # dist = np.linalg.norm(np.array([b2.x - b1.x, b2.y - b1.y]))
        # sum_radii = b1.radius + b2.radius
        # new_radius =  (dist - sum_radii) / 2.0
        # if new_radius < self.robot_radius:
        #     new_radius = self.robot_radius
        # new_bubble = Bubble((new_x, new_y), new_radius)
        new_bubble = Bubble((new_x, new_y), self.robot_radius)
        self.update_bubble_radius(new_bubble)
        self.bubbles.insert(index + 1, new_bubble)

    def remove_bubble(self, index):
        """
        Remove bubble at the specified index
        """
        if 0 <= index < len(self.bubbles):
            self.bubbles.pop(index)

    def update_bubble_radius(self, bubble: Bubble):
        """
        Update bubble radius based on clearance from obstacles.
        This method should be called whenever:
        1. A new bubble is created
        2. A bubble position is updated
        """
        clearance = self.compute_clearance(bubble)
        # Ensure clearance is finite and at least robot_radius
        if not np.isfinite(clearance):
            clearance = self.base_clearance  # Large but finite value for free space
        bubble.radius = max(clearance, self.robot_radius)  # ensure minimum radius

    def compute_clearance(self, bubble: Bubble):
        """
        Compute clearance d(b) of a bubble from obstacles in the environment.
        Clearance is defined as the minimum distance from the bubble's center to the nearest obstacle.
        """
        min_clearance = 5.0
        # center of the bubble in grid coordinates
        gx, gy = self.env.world_to_grid(bubble.x, bubble.y)
        
        # Search in a larger area (use d_safe or max expected radius)
        # search_radius = max(self.d_safe * 2, 3.0 * self.robot_radius)
        search_radius = 5.0
        rad_cells = int(search_radius / self.env.resolution)

        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                nx, ny = gx + dx, gy + dy
                
                # Check bounds
                if not self.env.is_valid_grid(nx, ny):
                    continue
                    
                if self.env.get_occupancy_grid(nx, ny) != 1:
                    continue

                ox, oy = self.env.grid_to_world(nx, ny)
                # Distance from bubble center to obstacle (not from surface)
                dist = np.linalg.norm(np.array([bubble.x - ox, bubble.y - oy]))
                if dist < min_clearance:
                    min_clearance = dist

        # If no obstacles found in search area, return a large but finite value
        if np.isinf(min_clearance):
            min_clearance = search_radius
        
        return min_clearance
    
    def compute_potential_field_weight(self, bubble: Bubble):
        """
        Compute rho(b) where b is a bubble using repulsive potential field
        """
        if bubble.radius >= self.d_safe:
            return 0.0
        return self.k * (1.0/bubble.radius - 1.0/self.d_safe)

    def check_overlap(self, b1: Bubble, b2: Bubble):
        """
        Check if two bubbles overlap.
        Returns True if they overlap (distance < sum of radii).
        """
        dist = np.linalg.norm(np.array([b1.x - b2.x, b1.y - b2.y]))
        # Bubbles overlap if distance is less than sum of radii
        return dist < (b1.radius + b2.radius)

    def contraction_force(self, b_prev: Bubble, b_curr: Bubble, b_next: Bubble):
        """
        Compute contraction force on the current bubble from its neighbors.
        """
        vec_prev = np.array([b_prev.x - b_curr.x, b_prev.y - b_curr.y])
        vec_next = np.array([b_next.x - b_curr.x, b_next.y - b_curr.y])

        dist_prev = np.linalg.norm(vec_prev)
        dist_next = np.linalg.norm(vec_next)

        f_contraction = np.zeros(2)
        if dist_prev > 1e-6:
            f_contraction += self.kc * (vec_prev / dist_prev)
        if dist_next > 1e-6:
            f_contraction += self.kc * (vec_next / dist_next)

        return f_contraction
    
    def repulsion_force(self, b_curr: Bubble):
        """
        Compute repulsion force on the bubble from obstacles.
        """
        # compute potential field weight
        rho = self.compute_potential_field_weight(b_curr)
        if rho >= self.rho0 or rho == 0.0:
            return np.zeros(2)
        
        # Compute gradient by numerical differentiation
        x = b_curr.x
        y = b_curr.y
        h = 0.01  # small fixed step size for numerical gradient
        
        # Create temporary bubbles at shifted positions
        bubble_x1 = Bubble((x + h, y, 0.0), self.robot_radius)
        bubble_x2 = Bubble((x - h, y, 0.0), self.robot_radius)
        bubble_y1 = Bubble((x, y + h, 0.0), self.robot_radius)
        bubble_y2 = Bubble((x, y - h, 0.0), self.robot_radius)
        
        # Update radii for temporary bubbles based on their positions
        self.update_bubble_radius(bubble_x1)
        self.update_bubble_radius(bubble_x2)
        self.update_bubble_radius(bubble_y1)
        self.update_bubble_radius(bubble_y2)
        
        # Compute potential field weights at shifted positions
        rho_x1 = self.compute_potential_field_weight(bubble_x1)
        rho_x2 = self.compute_potential_field_weight(bubble_x2)
        rho_y1 = self.compute_potential_field_weight(bubble_y1)
        rho_y2 = self.compute_potential_field_weight(bubble_y2)
        
        # Compute gradient: points in direction of increasing potential (toward obstacles)
        grad_rho = np.array([(rho_x1 - rho_x2) / (2 * h), (rho_y1 - rho_y2) / (2 * h)])

        # Repulsive force opposes gradient (away from obstacles)
        # When rho increases (approaching obstacle), we want negative force
        f_repulsion = - self.kr * (self.rho0 - rho) * grad_rho
        
        return f_repulsion

    def get_points(self):
        """Return the current points of the elastic band."""
        return [(bubble.x, bubble.y, bubble.theta) for bubble in self.bubbles]