import numpy as np
from scipy.ndimage import binary_dilation

class OccupancyGrid:
    """
    Two-layer occupancy grid:
    - static_grid  : user-drawn & permanent obstacles
    - dynamic_grid : moving obstacles (updated every frame)
    final occupancy = static OR dynamic
    """

    def __init__(self, width, height, resolution=1.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)

        self.static_grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.dynamic_grid = np.zeros((self.height, self.width), dtype=np.uint8)

    # Unified API — Legacy compatibility (IMPORTANT)
    def clear(self):
        """Clear ALL obstacles (static + dynamic)."""
        self.static_grid.fill(0)
        self.dynamic_grid.fill(0)

    def set_occupancy_world(self, x, y, value=1):
        gx, gy = self.world_to_grid(x, y)
        self.set_occupancy_grid(gx, gy, value)

    def set_occupancy_grid(self, gx, gy, value=1):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.static_grid[gy, gx] = 1 if value else 0

    def get_occupancy_grid(self, gx, gy):
        """Return final occupancy including static + dynamic."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return max(self.static_grid[gy, gx], self.dynamic_grid[gy, gx])
        return 1  # out of bounds = wall

    # Static obstacle painting
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

    # Dynamic obstacles
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

    # Useful API
    def get_final_grid(self):
        return np.maximum(self.static_grid, self.dynamic_grid)

    def world_to_grid(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution
