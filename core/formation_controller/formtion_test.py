import formation_controller
from formation_controller import Formation, Leader, Follower
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from pathlib import Path


shape = 'triangle'  # 'triangle' or 'pentagon'
# pentagon not working for now
if shape == 'pentagon':
    formation_graph = {
    "leader": {"F1", "F2"},
    "F1": {"leader", "F3"},
    "F2": {"leader", "F4"},
    "F3": {"F1", "F4"},
    "F4": {"F2", "F3"}
    }
    formation_polygon = {
        "leader": (0.0, 0.0),
        "F1": (-1.9021, -1.3820),
        "F3": (-1.1756, -3.6180),
        "F4": (1.1756, -3.6180),
        "F2": (1.9021, -1.3820)
    }
else:
    formation_graph = {
        "leader": {"F1", "F2"},
        "F1": {"leader", "F2"},
        "F2": {"leader", "F1"}
    }
    formation_polygon = {
        "leader": (0.0, 0.0),
        "F1": (2.0, 2.0),
        "F2": (1.0, -2.0)
    }

def main():
    # create the formation 
    formation = Formation(formation_graph, formation_polygon)
    # compute formation radius (enclosing circle)
    formation_radius = formation.compute_formation_radius()
    # distance = formation.dij
    # print("Interdistances:", distance)
    # print(f"Formation radius: {formation_radius:.2f}")
    # create a leader robot
    leader_robot = Leader(
        name="leader",
        initial_position=(0.0, 0.0),
        formation_base_radius=formation_radius
    )
    # create follower robots
    followers = {}
    for follower_name in formation_graph["leader"]:
        interdistances = {neighbor: formation.dij[(follower_name, neighbor)] for neighbor in formation_graph[follower_name]}
        # initial_pos_noise = np.random.normal(0, 0.1, 2)  # small noise
        initial_pos_noise = np.array([0.0, 0.0])  # increased noise for testing
        initial_position = (formation_polygon[follower_name][0] + initial_pos_noise[0], formation_polygon[follower_name][1] + initial_pos_noise[1])
        print(f"{follower_name} initial position: {initial_position}")
        print(f"{follower_name} interdistances: {interdistances}")
        follower_robot = Follower(
            name=follower_name,
            initial_position=initial_position,
            neighbors=formation_graph[follower_name],
            interdistances=interdistances
        )
        followers[follower_name] = follower_robot

    # main loop to simulate formation scaling
    num_iterations = 200
    leader_velocity = (-0.5, 0.0)  # Move along x-axis
    dt = 0.1 # time step
    trajectories = {name: [follower.position] for name, follower in followers.items()}
    trajectories["leader"] = [leader_robot.position]
    
    # Run simulation and collect data
    for iteration in range(num_iterations):
        scale_factor = leader_robot.compute_scale(iteration)
        for follower in followers.values():
            follower.scale_interdistances(scale_factor)
            follower.read_neighbors_positions({**followers, "leader": leader_robot})
            velocity = follower.controller()
            follower.move(velocity, dt)
            trajectories[follower.name].append(follower.position)
        
        # move leader
        leader_robot.move(leader_velocity, dt)  # Move leader forward
        trajectories["leader"].append(leader_robot.position)
        
        if iteration % 25 == 0:
            print(f"Iteration {iteration}: Leader at {leader_robot.position}, Scale factor: {scale_factor:.2f}")
    
    # Create animation and save it
    create_formation_animation(formation, trajectories, followers, formation_graph)
    
    # Plot final trajectories
    plt.figure(figsize=(10, 6))
    for name, traj in trajectories.items():
        traj = np.array(traj)
        plt.plot(traj[:,0], traj[:,1], label=name, linewidth=2)
    plt.legend()
    plt.title("Trajectories of Leader and Followers")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

    # plot interdistances over time
    plt.figure(figsize=(12, 6))
    
    # Collect all unique edges
    plotted_edges = set()
    
    for follower_name, follower in followers.items():
        follower_positions = np.array([pos[:2] for pos in trajectories[follower_name]])
        
        for neighbor in follower.neighbors:
            # Create edge tuple (sorted to avoid duplicates like (F1,F2) and (F2,F1))
            edge = tuple(sorted([follower_name, neighbor]))
            
            if edge in plotted_edges:
                continue
            plotted_edges.add(edge)
            
            # Get neighbor positions
            if neighbor == "leader":
                neighbor_positions = np.array([pos[:2] for pos in trajectories["leader"]])
            else:
                neighbor_positions = np.array([pos[:2] for pos in trajectories[neighbor]])
            
            # Calculate distances over time
            distances = np.linalg.norm(follower_positions - neighbor_positions, axis=1)
            
            # Plot
            plt.plot(distances, label=f"{follower_name} ↔ {neighbor}", linewidth=2)
    
    plt.title("Interdistances Over Time for All Neighbors")
    plt.xlabel("Time step")
    plt.ylabel("Distance")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def create_formation_animation(formation, trajectories, followers_dict, formation_graph, save_path=None):
    """
    Create animated visualization of formation movement.
    
    Args:
        save_path: Path to save animation. Extension determines format:
                   - .gif for animated GIF
                   - .mp4 for MP4 video (requires ffmpeg)
                   - None to only display
    """
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Convert trajectories to numpy arrays
    traj_arrays = {name: np.array(traj) for name, traj in trajectories.items()}
    
    # Determine plot limits
    all_x = np.concatenate([traj[:, 0] for traj in traj_arrays.values()])
    all_y = np.concatenate([traj[:, 1] for traj in traj_arrays.values()])
    margin = 0.5
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    # Initialize plot elements
    agent_plots = {}
    trail_plots = {}
    edge_plots = []
    
    # Leader
    agent_plots['leader'], = ax.plot([], [], 'ro', markersize=10, label='Leader')
    trail_plots['leader'], = ax.plot([], [], 'r-', alpha=0.3, linewidth=1)
    
    # Followers
    colors = ['blue', 'green', 'orange', 'purple']
    for idx, follower_name in enumerate(followers_dict.keys()):
        color = colors[idx % len(colors)]
        agent_plots[follower_name], = ax.plot([], [], 'o', color=color, markersize=8, label=follower_name)
        trail_plots[follower_name], = ax.plot([], [], '-', color=color, alpha=0.3, linewidth=1)
    
    # Formation edges
    for follower_name in followers_dict.keys():
        for neighbor in formation_graph[follower_name]:
            if follower_name < neighbor:  # Avoid duplicate edges
                line, = ax.plot([], [], 'k--', alpha=0.5, linewidth=1.5)
                edge_plots.append((line, follower_name, neighbor))
    
    # Text for iteration counter
    text = ax.text(0.02, 0.98, '', transform=ax.transAxes, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    ax.legend(loc='upper right')
    
    def init():
        for plot in agent_plots.values():
            plot.set_data([], [])
        for plot in trail_plots.values():
            plot.set_data([], [])
        for line, _, _ in edge_plots:
            line.set_data([], [])
        text.set_text('')
        return list(agent_plots.values()) + list(trail_plots.values()) + [line for line, _, _ in edge_plots] + [text]
    
    def animate(frame):
        # Update agent positions
        for name, plot in agent_plots.items():
            pos = traj_arrays[name][frame]
            plot.set_data([pos[0]], [pos[1]])
        
        # Update trails (show last 50 frames)
        trail_length = min(50, frame + 1)
        for name, plot in trail_plots.items():
            traj = traj_arrays[name][max(0, frame - trail_length):frame + 1]
            plot.set_data(traj[:, 0], traj[:, 1])
        
        # Update formation edges
        for line, agent1, agent2 in edge_plots:
            pos1 = traj_arrays[agent1][frame]
            pos2 = traj_arrays[agent2][frame]
            line.set_data([pos1[0], pos2[0]], [pos1[1], pos2[1]])
        
        # Update text
        text.set_text(f'Frame: {frame}/{len(traj_arrays["leader"])-1}')
        
        return list(agent_plots.values()) + list(trail_plots.values()) + [line for line, _, _ in edge_plots] + [text]
    
    # Create animation
    num_frames = len(traj_arrays['leader'])
    # Skip frames for faster animation (show every 2nd frame)
    frames = range(0, num_frames, 2)
    
    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                   interval=50, blit=True, repeat=True)
    
    plt.title('Distance-based Formation Control with Scaling (Holonomic particles for testing)')
    plt.tight_layout()
    
    # Save animation if path provided
    if save_path:
        save_path = Path(save_path)
        print(f"Saving animation to {save_path}...")
        
        if save_path.suffix == '.gif':
            # Save as GIF (no external dependencies needed)
            anim.save(save_path, writer='pillow', fps=20, dpi=100)
            print(f"Animation saved as GIF: {save_path}")
        elif save_path.suffix == '.mp4':
            # Save as MP4 (requires ffmpeg)
            try:
                anim.save(save_path, writer='ffmpeg', fps=20, dpi=150, bitrate=1800)
                print(f"Animation saved as MP4: {save_path}")
            except Exception as e:
                print(f"Error saving MP4 (ffmpeg may not be installed): {e}")
                # Fallback to GIF
                gif_path = save_path.with_suffix('.gif')
                print(f"Saving as GIF instead: {gif_path}")
                anim.save(gif_path, writer='pillow', fps=20, dpi=100)
        else:
            print(f"Unsupported format: {save_path.suffix}. Use .gif or .mp4")
    
    plt.show()

def visualize_formation(formation:Formation, leader_pos, followers_dict):
    plt.clf()
    # Plot leader
    plt.plot(leader_pos[0], leader_pos[1], 'ro', label='Leader')
    # Plot followers
    for follower_name, follower in followers_dict.items():
        plt.plot(follower.position[0], follower.position[1], 'bo', label=follower_name)
        # Draw lines to neighbors
        for neighbor in follower.neighbors:
            if neighbor == "leader":
                neighbor_pos = leader_pos
            else:
                neighbor_pos = followers_dict[neighbor].position
            plt.plot([follower.position[0], neighbor_pos[0]], [follower.position[1], neighbor_pos[1]], 'k--')
    plt.axis('equal')
    plt.legend()
    plt.pause(0.5)
if __name__ == "__main__":
    main()