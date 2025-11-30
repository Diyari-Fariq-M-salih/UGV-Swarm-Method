import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from elasticBand import elasticBand, Bubble

class SimpleEnvironment:
    """Simple 2D grid environment for testing."""
    
    def __init__(self, width=20.0, height=20.0, resolution=0.1):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_height))
    
    def add_obstacle(self, x, y, radius):
        """Add a circular obstacle at (x, y) with given radius."""
        gx, gy = self.world_to_grid(x, y)
        rad_cells = int(radius / self.resolution)
        
        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_valid_grid(nx, ny):
                    ox, oy = self.grid_to_world(nx, ny)
                    dist = np.linalg.norm(np.array([x - ox, y - oy]))
                    if dist <= radius:
                        self.occupancy_grid[nx, ny] = 1
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates."""
        x = gx * self.resolution
        y = gy * self.resolution
        return x, y
    
    def is_valid_grid(self, gx, gy):
        """Check if grid coordinates are valid."""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height
    
    def get_occupancy_grid(self, gx, gy):
        """Get occupancy value at grid coordinates."""
        if not self.is_valid_grid(gx, gy):
            return 1  # Treat out-of-bounds as occupied
        return self.occupancy_grid[gx, gy]

def visualize_eband(eband, env, title="Elastic Band"):
    """Visualize the elastic band with bubbles."""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Draw obstacles
    obstacle_x, obstacle_y = [], []
    for gx in range(env.grid_width):
        for gy in range(env.grid_height):
            if env.occupancy_grid[gx, gy] == 1:
                x, y = env.grid_to_world(gx, gy)
                obstacle_x.append(x)
                obstacle_y.append(y)
    
    ax.scatter(obstacle_x, obstacle_y, c='red', s=1, alpha=0.3, label='Obstacles')
    
    # Draw elastic band path
    path_x = [b.x for b in eband.bubbles]
    path_y = [b.y for b in eband.bubbles]
    ax.plot(path_x, path_y, 'b-', linewidth=2, label='Elastic Band')
    
    # Draw bubbles
    for i, bubble in enumerate(eband.bubbles):
        circle = Circle((bubble.x, bubble.y), bubble.radius, 
                       fill=False, edgecolor='green', linewidth=1.5, alpha=0.6)
        ax.add_patch(circle)
        
        # Mark bubble centers
        if i == 0:
            ax.plot(bubble.x, bubble.y, 'go', markersize=8, label='Start')
        elif i == len(eband.bubbles) - 1:
            ax.plot(bubble.x, bubble.y, 'ro', markersize=8, label='Goal')
        else:
            ax.plot(bubble.x, bubble.y, 'bo', markersize=4)
    
    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(title)
    plt.tight_layout()
    plt.show()

def test_eband():
    """Test elastic band planner with simple scenarios."""
    
    # Create environment
    env = SimpleEnvironment(width=20.0, height=20.0, resolution=0.1)
    
    # Test 1: Single obstacle
    print("Test 1: Single obstacle in the middle")
    env.add_obstacle(10.0, 10.0, 2.0)
    
    # Create initial path that goes around the obstacle
    start = (2.0, 10.0, 0.0)
    goal = (18.0, 10.0, 0.0)
    n_points = 25
    initial_path = []
    
    # Create a path that curves around the obstacle
    for i in range(n_points):
        t = i / (n_points - 1)
        x = start[0] + t * (goal[0] - start[0])
        # Add a curve to avoid the obstacle at (10, 10)
        if 0.3 < t < 0.7:
            # Curve upward around the obstacle
            curve_factor = np.sin((t - 0.3) / 0.4 * np.pi)
            y = start[1] + 3.0 * curve_factor
        else:
            y = start[1]
        initial_path.append((x, y, 0.0))
    
    # Initialize elastic band with adjusted parameters
    params = {
        'eband_params': {
            'kc': 0.5,          # Reduced contraction
            'kr': 5.0,          # Increased repulsion
            'k_potential_field': 2.0,
            'alpha': 0.03,      # Smaller step size
            'inflation_dist': 0.4,
            'rho0_tol': 0.3
        },
        'robot_params': {
            'radius': 0.3
        }
    }
    
    eband = elasticBand(initial_path, env, **params)
    
    # Visualize initial state
    visualize_eband(eband, env, "Initial Elastic Band - Test 1")
    
    # Run deformation for several iterations
    print("Running deformation...")
    N_iterations = 10
    for iteration in range(N_iterations):
        eband.deform_eband()
        print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
        if iteration % 10 == 9:
            print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
    
    # Visualize final state
    visualize_eband(eband, env, f"Final Elastic Band - Test 1 ({N_iterations} iterations)", )
    
    # Test 2: Add another obstacle
    print("\nTest 2: Adding another obstacle")
    env.add_obstacle(16.0, 8.0, 1.0)  # Changed position to below the path
    
    # Visualize before deformation
    visualize_eband(eband, env, "Elastic Band with Second Obstacle - Before Deformation")
    
    # Run more deformation iterations
    print("Running deformation with second obstacle...")
    for iteration in range(N_iterations):
        eband.deform_eband()
        print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
        if iteration % 10 == 9:
            print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
    
    # Visualize final state
    visualize_eband(eband, env, f"Final Elastic Band - Test 2 ({N_iterations} total iterations)")
    
    # Test 3: Move obstacle to path location
    print("\nTest 3: Moving obstacle to block the path")
    # Clear second obstacle and add it at new location
    env.occupancy_grid = np.zeros((env.grid_width, env.grid_height))
    env.add_obstacle(10.0, 10.0, 2.0)  # Re-add first obstacle
    env.add_obstacle(16.0, 9.0, 1.0)  # Add second obstacle directly on path
    
    # Visualize before deformation
    visualize_eband(eband, env, "Elastic Band with Obstacle on Path - Before Deformation")
    
    # Run more deformation iterations
    print("Running deformation with obstacle on path...")
    for iteration in range(N_iterations):
        eband.deform_eband()
        print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
        if iteration % 10 == 9:
            print(f"  Iteration {iteration + 1}: {len(eband.bubbles)} bubbles")
    
    # Visualize final state
    visualize_eband(eband, env, f"Final Elastic Band - Test 3 ({3*N_iterations} total iterations)")
    
    print("\nTest complete!")
    print(f"Final number of bubbles: {len(eband.bubbles)}")
    if len(eband.bubbles) > 1:
        path_length = sum(np.linalg.norm(np.array([eband.bubbles[i+1].x - eband.bubbles[i].x, 
                                                     eband.bubbles[i+1].y - eband.bubbles[i].y])) 
                         for i in range(len(eband.bubbles)-1))
        print(f"Path length: {path_length:.2f}")

if __name__ == "__main__":
    test_eband()
    plt.show()