import numpy as np
from scipy.ndimage import binary_dilation

class OccupancyGrid:
    """
    Two-layer occupancy grid:
    - static_grid           : user-visible obstacles
    - static_grid_inflated  : used internally for planning (INVISIBLE)
    - dynamic_grid          : moving obstacles (visible + used for planning)
    """

    def __init__(self, width, height, resolution=1.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)

        # Visible map
        self.static_grid = np.zeros((self.height, self.width), dtype=np.uint8)

        # Invisible inflated map (used only by planners)
        self.static_grid_inflated = np.zeros((self.height, self.width), dtype=np.uint8)

        # Dynamic obstacles
        self.dynamic_grid = np.zeros((self.height, self.width), dtype=np.uint8)

    # ---------------------------------------------------------
    # BASIC UTILITIES
    # ---------------------------------------------------------

    def clear(self):
        """Clear ALL obstacles (static + inflated + dynamic)."""
        self.static_grid.fill(0)
        self.static_grid_inflated.fill(0)
        self.dynamic_grid.fill(0)

    def world_to_grid(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    # ---------------------------------------------------------
    # SETTING OBSTACLES
    # ---------------------------------------------------------

    def set_occupancy_world(self, x, y, value=1):
        gx, gy = self.world_to_grid(x, y)
        self.set_occupancy_grid(gx, gy, value)

    def set_occupancy_grid(self, gx, gy, value=1):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.static_grid[gy, gx] = 1 if value else 0
            # Must update inflated grid because visible obstacles changed
            self.recompute_inflation()

    def paint_static_square(self, cx, cy, size, value=1):
        """Visible painted obstacle square."""
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
        
        # Recompute inflated map
        self.recompute_inflation()

    # ---------------------------------------------------------
    # DYNAMIC OBSTACLES
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # GRID GETTERS
    # ---------------------------------------------------------

    def get_final_grid(self):
        """
        Return VISIBLE grid (static + dynamic).
        IMPORTANT: inflated grid is NOT shown visually.
        """
        return np.maximum(self.static_grid, self.dynamic_grid)

    def get_occupancy_grid(self, gx, gy):
        """
        Return occupancy for PLANNING.
        Uses inflated grid + dynamic obstacles.
        """
        if 0 <= gx < self.width and 0 <= gy < self.height:
            inflated = self.static_grid_inflated[gy, gx]
            dynamic = self.dynamic_grid[gy, gx]
            return max(inflated, dynamic)
        return 1  # out of bounds = wall

    # ---------------------------------------------------------
    # OBSTACLE INFLATION (INVISIBLE SAFETY MARGIN)
    # ---------------------------------------------------------

    def recompute_inflation(self):
        """
        Recompute inflated grid after changes to static_grid.
        Default inflation radius preserved.
        If no inflate_obstacles() was called yet, leave inflated empty.
        """
        # If inflated grid is zero and user never set inflation → do nothing
        if not hasattr(self, "_inflation_radius") or self._inflation_radius <= 0:
            self.static_grid_inflated = self.static_grid.copy()
            return

        self.inflate_obstacles(self._inflation_radius)

    def inflate_obstacles(self, radius=1.0):
        """
        Inflate obstacles by N meters for planning safety.
        Inflation is invisible — only affects path planning.
        """
        self._inflation_radius = radius

        cells = int(radius / self.resolution)
        if cells <= 0:
            self.static_grid_inflated = self.static_grid.copy()
            return

        struct = np.ones((2 * cells + 1, 2 * cells + 1), dtype=np.uint8)

        inflated = binary_dilation(self.static_grid, structure=struct)

        # Store inflated grid for planning ONLY
        self.static_grid_inflated = inflated.astype(np.uint8)
