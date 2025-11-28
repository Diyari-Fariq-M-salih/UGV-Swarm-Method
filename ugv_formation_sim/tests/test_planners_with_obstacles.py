import matplotlib.pyplot as plt

from core.planners.astar import AStarPlanner
from core.planners.rrt import RRTPlanner
from core.planners.potential_field import PotentialFieldPlanner

print("Testing A*, RRT, and Potential Field planners...")
# Obstacles: mix of circle + rectangl
obstacles = [
    ("circle", 4, 0, 1.0),        # circle at (4, 0) radius 1
    ("rect", 6, -1, 7.5, 1.0)     # rectangle from x=[6,7.5], y=[-1,1]
]

start = (0, 3)
goal = (10, 0)

# Instantiate planner
planners = {
    "A*": AStarPlanner(grid_resolution=0.2),
    "RRT": RRTPlanner(step_size=0.3, max_iter=3000, goal_bias=0.10),
    "Potential Field": PotentialFieldPlanner(step=0.1)
}
# Run planners and collect result
paths = {}
for name, planner in planners.items():
    print(f"\nRunning {name} planner...")
    path = planner.plan(start, goal, obstacles)
    paths[name] = path
    print(f"{name} produced {len(path)} waypoints")
# Plot result
plt.figure(figsize=(10, 6))

# Plot planners
for name, path in paths.items():
    if len(path) > 0:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        plt.plot(xs, ys, label=name)

# Draw obstacles
ax = plt.gca()

for obs in obstacles:
    if obs[0] == "circle":
        _, cx, cy, r = obs
        circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
        ax.add_patch(circ)

    elif obs[0] == "rect":
        _, x1, y1, x2, y2 = obs
        rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, color='m', alpha=0.3)
        ax.add_patch(rect)

plt.legend()
plt.title("A*, RRT, and Potential Field Path Planning")
plt.axis("equal")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()
