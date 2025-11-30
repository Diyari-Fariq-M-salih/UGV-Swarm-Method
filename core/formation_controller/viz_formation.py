
from formation_controller import Formation, formationController
import matplotlib.pyplot as plt
import numpy as np

formation_graph = {
    "leader": {"F1", "F2"},
    "F1": {"leader", "F2"},
    "F2": {"leader", "F1"}
}
formation_polygon = {
    "leader": (0, 0),
    "F1": (5, 2),
    "F2": (5, -2)
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
    scale_factor = 0.8
    fc.scale_interdistances(scale_factor)
    print(f"\nAfter scaling by factor {scale_factor}:")
    fc.visualize_formation()
    plt.show()
    print("Bearings (degrees):")
    for (i, j), bearing in fc.bij.items():
        print(f"  From {j} to {i}: {bearing:.2f}°")

if __name__ == "__main__":
    main()