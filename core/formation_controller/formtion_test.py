import formation_controller
from formation_controller import Formation, Leader, Follower
import matplotlib.pyplot as plt
import numpy as np

formation_graph = {
    "leader": {"F1", "F2"},
    "F1": {"leader", "F2"},
    "F2": {"leader", "F1"}
}
formation_polygon = {
    "leader": (0.0, 0.0),
    "F1": (5.0, 2.0),
    "F2": (4.0, -2.0)
}

def main():
    # create the formation 
    formation = Formation(formation_graph, formation_polygon)
    # compute formation radius (enclosing circle)
    formation_radius = formation.compute_formation_radius()
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
    num_iterations = 40
    plt.figure()
    leader_velocity = (1.0, 0.0)  # Move along x-axis
    dt = 0.01 # time step
    trajectories = {name: [follower.position] for name, follower in followers.items()}
    trajectories["leader"] = [leader_robot.position]
    for iteration in range(num_iterations):
        # compute scale factor (varies every 10 iterations for testing)
        scale_factor = leader_robot.compute_scale(iteration)
        for follower in followers.values():
            follower.scale_interdistances(scale_factor)
            follower.read_neighbors_positions({**followers, "leader": leader_robot})
            velocity = follower.controller()
            follower.move(velocity, dt)
            trajectories[follower.name].append(follower.position)
        # visualize formation every 10 iterations
        if iteration % 10 == 0:
            print(f"Iteration {iteration}: Leader at {leader_robot.position}, Scale factor: {scale_factor:.2f}")
            visualize_formation(formation, leader_robot.position, followers)
            plt.title(f"Formation at Iteration {iteration}")
            plt.show()
        # move leader
        leader_robot.move(leader_velocity, dt)  # Move leader forward
        trajectories["leader"].append(leader_robot.position)
    # plot trajectories
    plt.clf()
    for name, traj in trajectories.items():
        traj = np.array(traj)
        plt.plot(traj[:,0], traj[:,1], label=name)
    plt.legend()
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