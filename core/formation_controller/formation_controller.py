from networkx import edges
import numpy as np
import matplotlib.pyplot as plt

# # Define formation as undirected graph
# formation_graph = {
#     "leader": {"F1", "F2"},
#     "F1": {"leader", "F3"},
#     "F2": {"leader"},
#     "F3": {"F1"}
# }
# # Define polygon of desired relative positions (to extract bearing and nominal distance)
# formation_polygon = {
#     "leader": (0, 0),
#     "F1": (5, 2),
#     "F2": (5, -2),
#     "F3": (10, 2)
# }

import numpy as np

class Formation:
    def __init__(self, graph, polygon):
        self.graph = graph                      # adjacency sets
        self.polygon = polygon                  # dict: node → (x, y)
        self.bij = {}                           # dict: (i,j) → bearing_ij
        self.compute_bearings()
        self.dij = {}                           # dict: (i,j) → distance_ij
        self.compute_distances()
        self.centroid = self.compute_centroid()

    def compute_centroid(self):
        xs = [pos[0] for pos in self.polygon.values()]
        ys = [pos[1] for pos in self.polygon.values()]
        centroid_x = sum(xs) / len(xs)
        centroid_y = sum(ys) / len(ys)
        return (centroid_x, centroid_y)
    
    def compute_formation_radius(self):
        max_distance = 0
        for pos in self.polygon.values():
            distance = np.linalg.norm(np.array(pos) - np.array(self.centroid))
            if distance > max_distance:
                max_distance = distance
        return max_distance
    
    def compute_bearings(self):
        for node in self.graph:
            for neighbor in self.graph[node]:
                pos_node = self.polygon[node]
                pos_neighbor = self.polygon[neighbor]

                dx = pos_neighbor[0] - pos_node[0]
                dy = pos_neighbor[1] - pos_node[1]

                bearing_ij = np.degrees(np.arctan2(dy, dx))   # i → j

                # store directed bearings
                self.bij[(node, neighbor)] = bearing_ij
                self.bij[(neighbor, node)] = (bearing_ij + 180) % 360

    def compute_distances(self):
        self.dij = {}  # dict: (i,j) → distance_ij
        for node in self.graph:
            for neighbor in self.graph[node]:
                pos_node = self.polygon[node]
                pos_neighbor = self.polygon[neighbor]

                distance_ij = np.linalg.norm(np.array(pos_neighbor) - np.array(pos_node))

                # store directed distances
                self.dij[(node, neighbor)] = distance_ij
                self.dij[(neighbor, node)] = distance_ij
    
    def scale_positions(self, scale_factor):
        """Scale the formation polygon by a given factor."""
        for robot in self.polygon:
            x, y = self.polygon[robot]
            self.polygon[robot] = (x * scale_factor, y * scale_factor)
        # Recompute distances after scaling
        self.compute_distances()
        # Recompute bearings after scaling
        self.compute_bearings()

    def scale_interdistances(self, scale_factor):
        """Scale only the inter-robot distances by a given factor, keeping bearings."""
        new_polygon = {}
        for robot in self.polygon:
            x, y = self.polygon[robot]
            distance = np.linalg.norm(np.array([x, y]))
            bearing = np.degrees(np.arctan2(y, x))
            new_distance = distance * scale_factor
            new_x = new_distance * np.cos(np.radians(bearing))
            new_y = new_distance * np.sin(np.radians(bearing))
            new_polygon[robot] = (new_x, new_y)
        self.polygon = new_polygon
        # Recompute distances after scaling
        self.compute_distances()
        # Recompute bearings after scaling
        self.compute_bearings()

    def visualize_formation(self):
        """Visualize the formation given a dictionary of positions."""
        # plt.figure(figsize=(8, 8))
        for robot, position in self.polygon.items():
            plt.plot(position[0], position[1], 'o', label=robot)
            plt.text(position[0] + 0.1, position[1] + 0.1, robot)
                    
        for robot, neighbors in self.graph.items():
            for neighbor in neighbors:
                pos_robot = self.polygon[robot]
                pos_neighbor = self.polygon[neighbor]
                plt.plot([pos_robot[0], pos_neighbor[0]], [pos_robot[1], pos_neighbor[1]], 'k--', alpha=0.5)

        plt.title("Robot Formation")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.axis('equal')
        plt.grid()
        plt.legend()
        plt.show()

class Leader():
    """A placeholder leader robot class."""
    
    def __init__(self, name:str, initial_position, formation_base_radius):
        self.name = name
        self.position = initial_position
        self.formation_base_radius = formation_base_radius
        # create virtual radii for scaling formation
        # self.radii = [formation_base_radius * s for s in [1.0, 0.8, 0.6, 0.4]]

    # simulate robot movement (single integrator model)
    def move(self, velocity, dt):
        # """Placeholder move method for the leader robot."""
        # # linear velocity 
        # theta = self.position[2]
        # V = velocity[0]
        # # angular velocity
        # omega = velocity[1]
        # dx = V * np.cos(theta) * dt 
        # dy = V * np.sin(theta) * dt
        # dtheta = omega * dt
        # self.position = (self.position[0] + dx, self.position[1] + dy, self.position[2] + dtheta)
        dx = velocity[0] * dt
        dy = velocity[1] * dt   
        self.position = (self.position[0] + dx, self.position[1] + dy)

    def compute_scale(self, iteration_num):
        """Compute scale factor based on iteration number."""
        # index = (iteration_num // 2) % len(self.radii)
        # scale_factor = self.radii[index] / self.formation_base_radius
        # linearly decrease scale factor from 1.0 to 0.4 over 40 iterations
        max_iterations = 40
        scale_factor = 1.0 - 0.6 * (iteration_num / max_iterations)
        return scale_factor

    # def scale_formation(self, formation:Formation):
    #     """Scale formation based on distance to farthest robot in formation."""
    #     scale_factor = self.compute_scale(formation)
    #     formation.scale_interdistances(scale_factor)

    # def compute_scale(self, formation:Formation):
    #     """Compute scale factor based on distance to farthest robot in formation."""
    #     leader_pos = np.array(self.position[0:2])
    #     max_distance = 0
    #     for pos in formation.polygon.values():
    #         rel_pos = np.array(pos)
    #         distance = np.linalg.norm(rel_pos - leader_pos)
    #         if distance > max_distance:
    #             max_distance = distance
    #     if max_distance == 0:
    #         return 1.0
    #     scale_factor = 1.0 / max_distance
    #     return scale_factor


class Follower():
    """A placeholder follower robot class."""
    
    def __init__(self, name:str, initial_position, neighbors, interdistances):
        self.name = name
        self.neighbors = neighbors
        self.base_interdistances = interdistances  # dict: neighbor → distance
        self.dij_ref = interdistances.copy()  # dict: neighbor → distance
        self.position = initial_position
        self.neighbor_positions = {neighbor: np.array([0.0, 0.0]) for neighbor in neighbors}
    # # getter to be used with other followers
    # def get_position(self):
    #     return self.position

    def scale_interdistances(self, scale_factor):
        for dj in self.neighbors:
            # scale reference distances
            self.dij_ref[dj] =  self.base_interdistances[dj] * scale_factor

    def read_neighbors_positions(self, neighbors_dict):
        """
        Read positions of neighbors from a dictionary of robot objects.
        neighbors_dict: dict mapping neighbor names to robot objects (Leader or Follower)
        """
        for neighbor in self.neighbors:
            if neighbor in neighbors_dict:
                # Use position[:2] to get (x, y) position from (x, y, theta)
                self.neighbor_positions[neighbor] = np.array(neighbors_dict[neighbor].position[:2])
                # print(f"{self.name} read position of {neighbor}: {self.neighbor_positions[neighbor]}")
            else:
                print(f"Warning: Neighbor {neighbor} not found in neighbors_dict")
    
    def move(self, velocity, dt):
        dx = velocity[0] * dt
        dy = velocity[1] * dt   
        self.position = (self.position[0] + dx, self.position[1] + dy)
    
    def controller(self):
        """Placeholder controller method for the follower robot."""
        # Simple proportional controller to maintain distances
        Kp = 2.0
        vx, vy = 0.0, 0.0
        for neighbor in self.neighbors:
            neighbor_pos = np.array(self.neighbor_positions[neighbor])
            my_pos = np.array(self.position)
            direction = my_pos - neighbor_pos
            distance = np.linalg.norm(direction)
            if distance > 1e-6:
                direction_normalized = direction / distance
                error = distance - self.dij_ref[neighbor]
                vx -= Kp * error * direction_normalized[0]
                vy -= Kp * error * direction_normalized[1]
            else:
                print(f"Warning: {self.name} and {neighbor} are at the same position.")
        return (vx, vy)

class formationController():
    """A placeholder formation controller class."""
    
    def __init__(self, formation:Formation, **params):
        self.bearings = formation.bij
        self.distances = formation.dij
