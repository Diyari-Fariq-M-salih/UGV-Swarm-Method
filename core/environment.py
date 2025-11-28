import numpy as np
from scipy.ndimage import binary_dilation


class OccupancyGrid:
    """
    A simple binary occupancy grid with:
    - set/get occupancy
    - inflation
    - world/grid coordinate transforms
    """

    def __init__(self, width, height, resolution=1.0):
        """
        width, height = size in world units
        resolution = meters per grid cell
        """
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)

        # 2D array: 0 = free, 1 = occupied
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

        # world coordinates go from 0..width, 0..height

    # Occupancy access
    def set_occupancy_world(self, x, y, value=1):
        """Set occupancy using world coordinates (floats)."""
        gx, gy = self.world_to_grid(x, y)
        self.set_occupancy_grid(gx, gy, value)

    def set_occupancy_grid(self, gx, gy, value=1):
        """Set occupancy using grid coordinates (ints)."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.grid[gy, gx] = 1 if value else 0

    def get_occupancy_world(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        return self.get_occupancy_grid(gx, gy)

    def get_occupancy_grid(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx]
        return 1  # out-of-bounds = wall

    # Bulk obstacle painting
    def paint_square(self, cx, cy, size, value=1):
        """
        Paint a square obstacle centered at world (cx, cy) of side length `size`.
        """
        half = size / 2.0
        x_min, y_min = cx - half, cy - half
        x_max, y_max = cx + half, cy + half

        gx_min, gy_min = self.world_to_grid(x_min, y_min)
        gx_max, gy_max = self.world_to_grid(x_max, y_max)

        gx_min = max(gx_min, 0)
        gy_min = max(gy_min, 0)
        gx_max = min(gx_max, self.width - 1)
        gy_max = min(gy_max, self.height - 1)

        self.grid[gy_min:gy_max+1, gx_min:gx_max+1] = 1 if value else 0

    # Inflation (expands walls to create safety margin)
    def inflate(self, radius_cells):
        """
        Inflate the occupancy grid by `radius_cells`.
        """
        if radius_cells <= 0:
            return

        struct = np.ones((radius_cells*2+1, radius_cells*2+1), dtype=np.uint8)
        dilated = binary_dilation(self.grid, structure=struct)
        self.grid = dilated.astype(np.uint8)

    # Coordinate transforms
    def world_to_grid(self, x, y):
        """
        Convert world coordinates (meters) to grid integers.
        """
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """
        Convert grid coordinates to world coordinates (center of cell).
        """
        x = (gx + 0.5) * self.resolution
        y = (gy + 0.5) * self.resolution
        return x, y

    # Utility
    def inside_world(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def inside_grid(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def clear(self):
        """Reset the grid."""
        self.grid.fill(0)
