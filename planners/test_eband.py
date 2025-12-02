import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from elasticBand import elasticBand, Bubble
from pathlib import Path

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

def create_eband_animation(eband, env, obstacle_trajectory, save_path=None):
    """
    Create animation of elastic band adapting to moving obstacle.
    
    Args:
        eband: ElasticBand object
        env: Environment object
        obstacle_trajectory: List of (x, y, radius) tuples for obstacle positions
        save_path: Optional path to save animation (.gif or .mp4)
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Store bubble states for each frame
    bubble_history = []
    
    # Simulate and collect data
    print("Simulating elastic band with moving obstacle...")
    for frame_idx, (obs_x, obs_y, obs_r) in enumerate(obstacle_trajectory):
        # Update environment with new obstacle position
        env.occupancy_grid = np.zeros((env.grid_width, env.grid_height))
        env.add_obstacle(10.0, 10.0, 2.0)  # Static obstacle
        env.add_obstacle(obs_x, obs_y, obs_r)  # Moving obstacle
        
        # Run elastic band deformation
        eband.deform_eband()
        
        # Store current bubble state
        bubble_state = [(b.x, b.y, b.radius) for b in eband.bubbles]
        bubble_history.append({
            'bubbles': bubble_state,
            'obstacle': (obs_x, obs_y, obs_r)
        })
        
        if frame_idx % 5 == 0:
            print(f"  Frame {frame_idx}/{len(obstacle_trajectory)}: {len(eband.bubbles)} bubbles")
    
    print("Creating animation...")
    
    # Set up plot limits
    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    # Initialize plot elements
    obstacle_scatter = ax.scatter([], [], c='red', s=1, alpha=0.5, label='Static Obstacles')
    moving_obstacle = Circle((0, 0), 1, fill=True, color='darkred', alpha=0.5, label='Moving Obstacle')
    # static obstacle with same style but different color
    static_obstacle = Circle((10.0, 10.0), 2.0, fill=True, color='red', alpha=0.3, label='Static Obstacle')
    ax.add_patch(static_obstacle)
    ax.add_patch(moving_obstacle)
    
    path_line, = ax.plot([], [], 'b-', linewidth=2, label='Elastic Band')
    bubble_circles = []
    start_point, = ax.plot([], [], 'go', markersize=10, label='Start')
    goal_point, = ax.plot([], [], 'ro', markersize=10, label='Goal')
    bubble_centers, = ax.plot([], [], 'bo', markersize=4)
    
    # Text for frame counter
    text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                   verticalalignment='top', fontsize=12,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    ax.legend(loc='upper right')
    ax.set_title('Elastic Band Adapting to Moving Obstacle')
    
    def init():
        obstacle_scatter.set_offsets(np.empty((0, 2)))
        moving_obstacle.center = (0, 0)
        path_line.set_data([], [])
        start_point.set_data([], [])
        goal_point.set_data([], [])
        bubble_centers.set_data([], [])
        text.set_text('')
        return [obstacle_scatter, moving_obstacle, path_line, start_point, 
                goal_point, bubble_centers, text]
    
    def animate(frame):
        state = bubble_history[frame]
        bubbles = state['bubbles']
        obs_x, obs_y, obs_r = state['obstacle']
        
        # Draw static obstacles
        obstacle_points = []
        for gx in range(0, env.grid_width, 5):  # Sample for performance
            for gy in range(0, env.grid_height, 5):
                if env.occupancy_grid[gx, gy] == 1:
                    x, y = env.grid_to_world(gx, gy)
                    # Only show static obstacle (not moving one)
                    if np.sqrt((x - obs_x)**2 + (y - obs_y)**2) > obs_r + 0.5:
                        obstacle_points.append([x, y])
        
        if obstacle_points:
            obstacle_scatter.set_offsets(obstacle_points)
        
        # Update moving obstacle
        moving_obstacle.center = (obs_x, obs_y)
        moving_obstacle.radius = obs_r
        
        # Update elastic band path
        path_x = [b[0] for b in bubbles]
        path_y = [b[1] for b in bubbles]
        path_line.set_data(path_x, path_y)
        
        # Update start and goal
        start_point.set_data([bubbles[0][0]], [bubbles[0][1]])
        goal_point.set_data([bubbles[-1][0]], [bubbles[-1][1]])
        
        # Update bubble centers (excluding start and goal)
        if len(bubbles) > 2:
            centers_x = [b[0] for b in bubbles[1:-1]]
            centers_y = [b[1] for b in bubbles[1:-1]]
            bubble_centers.set_data(centers_x, centers_y)
        
        # Remove old bubble circles
        for circle in bubble_circles:
            circle.remove()
        bubble_circles.clear()
        
        # Draw new bubble circles
        for i, (bx, by, br) in enumerate(bubbles):
            if i == 0 or i == len(bubbles) - 1:
                continue  # Skip start and goal
            circle = Circle((bx, by), br, fill=False, 
                          edgecolor='green', linewidth=1.5, alpha=0.6)
            ax.add_patch(circle)
            bubble_circles.append(circle)
        
        # Update text
        text.set_text(f'Frame: {frame}/{len(bubble_history)-1}\n'
                     f'Bubbles: {len(bubbles)}\n'
                     f'Obstacle: ({obs_x:.1f}, {obs_y:.1f})')
        
        return [obstacle_scatter, moving_obstacle, path_line, start_point, 
                goal_point, bubble_centers, text] + bubble_circles
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, init_func=init, 
                                   frames=len(bubble_history),
                                   interval=100, blit=True, repeat=True)
    
    plt.tight_layout()
    
    # Save animation if path provided
    if save_path:
        save_path = Path(save_path)
        print(f"Saving animation to {save_path}...")
        
        if save_path.suffix == '.gif':
            anim.save(save_path, writer='pillow', fps=10, dpi=100)
            print(f"Animation saved as GIF: {save_path}")
        elif save_path.suffix == '.mp4':
            try:
                anim.save(save_path, writer='ffmpeg', fps=10, dpi=150, bitrate=1800)
                print(f"Animation saved as MP4: {save_path}")
            except Exception as e:
                print(f"Error saving MP4: {e}")
                gif_path = save_path.with_suffix('.gif')
                print(f"Saving as GIF instead: {gif_path}")
                anim.save(gif_path, writer='pillow', fps=10, dpi=100)
        else:
            print(f"Unsupported format: {save_path.suffix}. Use .gif or .mp4")
    
    plt.show()

def test_eband():
    """Test elastic band planner with animated moving obstacle."""
    
    # Create environment
    env = SimpleEnvironment(width=20.0, height=20.0, resolution=0.1)
    
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
            'rho0_tol': 0.3,
            'base_clearance': 1.5
        },
        'robot_params': {
            'radius': 0.3
        }
    }
    
    # Setup for animation: Obstacle moving from (14.0, 7.5) to (14.0, 9.0)
    print("Setting up elastic band animation with moving obstacle...")
    
    # Reset environment with initial obstacles
    env.add_obstacle(10.0, 10.0, 2.0)  # Static obstacle
    env.add_obstacle(14.0, 7.5, 1.0)   # Initial position of moving obstacle
    
    # Initialize elastic band
    eband = elasticBand(initial_path, env, **params)
    
    # Create obstacle trajectory (moving from y=7.5 to y=9.0)
    n_frames = 100  # Number of frames
    obstacle_trajectory = []
    for i in range(n_frames):
        t = i / (n_frames - 1)
        obs_y = 7.5 + t * (18.0 - 7.5)  # Linear interpolation
        obstacle_trajectory.append((14.0, obs_y, 1.0))
    
    # Create output directory
    output_dir = Path(__file__).parent.parent / 'logs'
    output_dir.mkdir(exist_ok=True)
    
    # Create and save animation
    animation_path = output_dir / 'eband_moving_obstacle.gif'
    create_eband_animation(eband, env, obstacle_trajectory, save_path=animation_path)
    
    print("\nAnimation complete!")
    print(f"Final number of bubbles: {len(eband.bubbles)}")
    if len(eband.bubbles) > 1:
        path_length = sum(np.linalg.norm(np.array([eband.bubbles[i+1].x - eband.bubbles[i].x, 
                                                     eband.bubbles[i+1].y - eband.bubbles[i].y])) 
                         for i in range(len(eband.bubbles)-1))
        print(f"Path length: {path_length:.2f}")

if __name__ == "__main__":
    test_eband()