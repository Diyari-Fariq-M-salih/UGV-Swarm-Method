
from formation_controller import Formation, formationController
import matplotlib.pyplot as plt
import numpy as np

# pentagon formation with 5 robots

shape = 'pentagon'  # 'triangle' or 'pentagon'
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
    """Test the formationController class."""
    fc = Formation(formation_graph, formation_polygon)
    print("Bearings (degrees):")
    for (i, j), bearing in fc.bij.items():
        print(f"  From {j} to {i}: {bearing:.2f}°")
    # Visualize the formation
    fc.visualize_formation()
    plt.show()
    # Test scaling the formation
    scale_factor = 0.9
    fc.scale_positions(scale_factor)
    print(f"\nAfter scaling by factor {scale_factor}:")
    fc.visualize_formation()
    plt.show()
    print("Bearings (degrees):")
    for (i, j), bearing in fc.bij.items():
        print(f"  From {j} to {i}: {bearing:.2f}°")


if __name__ == "__main__":
    main()