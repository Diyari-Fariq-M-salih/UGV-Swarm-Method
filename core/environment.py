import numpy as np
from scipy.ndimage import binary_dilation

class OccupancyGrid:
    """
    Occupancy grid with separate static and inflated layers.
    Dynamic obstacles are drawn separately and do NOT appear
    in the imshow grid to avoid visual artifacts.
    """
    def __init__(self, width, height, resolution=1.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)

        # Visible static obstacles
        self.static_grid = np.zeros((self.height, self.width), dtype=np.uint8)

        # Invisible inflated obstacles (for planning)
        self.static_grid_inflated = np.zeros((self.height, self.width), dtype=np.uint8)

        # Dynamic obstacles are tracked but NOT drawn in base grid
        self.dynamic_grid = np.zeros((self.height, self.width), dtype=np.uint8)

        self._inflation_radius = 0.0

    def clear(self):
        self.static_grid.fill(0)
        self.static_grid_inflated.fill(0)
        self.dynamic_grid.fill(0)

    def world_to_grid(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    def set_occupancy_world(self, x, y, value=1):
        gx, gy = self.world_to_grid(x, y)
        self.set_occupancy_grid(gx, gy, value)

    def set_occupancy_grid(self, gx, gy, value=1):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.static_grid[gy, gx] = 1 if value else 0
            self.recompute_inflation()

    def paint_static_square(self, cx, cy, size, value=1):
        half = size / 2
        x_min, y_min = cx - half, cy - half
        x_max, y_max = cx + half, cy + half

        gx_min, gy_min = self.world_to_grid(x_min, y_min)
        gx_max, gy_max = self.world_to_grid(x_max, y_max)

        gx_min = max(gx_min, 0)
        gy_min = max(gy_min, 0)
        gx_max = min(gx_max, self.width - 1)
        gy_max = min(gy_max, self.height - 1)

        self.static_grid[gy_min:gy_max+1, gx_min:gx_max+1] = 1 if value else 0
        self.recompute_inflation()

    def clear_dynamic(self):
        self.dynamic_grid.fill(0)

    def paint_dynamic_square(self, cx, cy, size):
        half = size / 2
        x_min, y_min = cx - half, cy - half
        x_max, y_max = cx + half, cy + half

        gx_min, gy_min = self.world_to_grid(x_min, y_min)
        gx_max, gy_max = self.world_to_grid(x_max, y_max)

        gx_min = max(gx_min, 0)
        gy_min = max(gy_min, 0)
        gx_max = min(gx_max, self.width - 1)
        gy_max = min(gy_max, self.height - 1)

        self.dynamic_grid[gy_min:gy_max+1, gx_min:gx_max+1] = 1

    # ============================================================
    # IMPORTANT FIX: dynamic obstacles DO NOT appear in base grid
    # ============================================================
    def get_final_grid(self):
        """Only static map is shown visually."""
        return self.static_grid

    def get_occupancy_grid(self, gx, gy):
        """Planning uses inflated static + dynamic obstacles."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            base = self.static_grid_inflated[gy, gx]
            return max(base, self.dynamic_grid[gy, gx])
        return 1

    def recompute_inflation(self):
        if self._inflation_radius <= 0:
            self.static_grid_inflated = self.static_grid.copy()
        else:
            self.inflate_obstacles(self._inflation_radius)

    def inflate_obstacles(self, radius=1.0):
        self._inflation_radius = radius
        cells = int(radius / self.resolution)
        if cells <= 0:
            self.static_grid_inflated = self.static_grid.copy()
            return

        struct = np.ones((2 * cells + 1, 2 * cells + 1), dtype=np.uint8)
        inflated = binary_dilation(self.static_grid, structure=struct)
        self.static_grid_inflated = inflated.astype(np.uint8)
