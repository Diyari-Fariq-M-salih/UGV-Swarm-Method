import numpy as np
from scipy.ndimage import binary_dilation

class OccupancyGrid:
    """
    Simple binary occupancy grid:
    - 0 = free
    - 1 = static or dynamic obstacle
    """

    def __init__(self, width, height, resolution=1.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    # Occupancy setters/getters
    def set_occupancy_world(self, x, y, value=1):
        gx, gy = self.world_to_grid(x, y)
        self.set_occupancy_grid(gx, gy, value)

    def set_occupancy_grid(self, gx, gy, value=1):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.grid[gy, gx] = 1 if value else 0

    def get_occupancy_grid(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx]
        return 1  # out of bounds = obstacle

    # Square painting
    def paint_square(self, cx, cy, size, value=1):
        half = size / 2
        x_min, y_min = cx - half, cy - half
        x_max, y_max = cx + half, cy + half

        gx_min, gy_min = self.world_to_grid(x_min, y_min)
        gx_max, gy_max = self.world_to_grid(x_max, y_max)

        gx_min = max(gx_min, 0)
        gy_min = max(gy_min, 0)
        gx_max = min(gx_max, self.width - 1)
        gy_max = min(gy_max, self.height - 1)

        self.grid[gy_min:gy_max+1, gx_min:gx_max+1] = 1 if value else 0

    # Inflation
    def inflate(self, radius_cells):
        if radius_cells <= 0:
            return

        struct = np.ones((radius_cells * 2 + 1,
                          radius_cells * 2 + 1), dtype=np.uint8)
        self.grid = binary_dilation(self.grid, structure=struct).astype(np.uint8)

    # Transforms
    def world_to_grid(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    def clear(self):
        self.grid.fill(0)
