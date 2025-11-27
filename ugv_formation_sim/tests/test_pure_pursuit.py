import time
import math

from core.ugv_model import UGV
from core.controllers.trajectory_tracking import PurePursuitController

# 1. Create a simple curved reference path
path = []

# Straight section
for x in range(12):
    path.append((x * 0.4, 0))

# Curve upward
for angle in range(25):
    theta = math.radians(angle * 4)
    x = 4 + 2 * math.cos(theta)
    y = 2 * math.sin(theta)      # lifts curve into positive y
    path.append((x, y))

# 2. Initialize robot and controller
robot = UGV(x=0, y=0, theta=0)
controller = PurePursuitController(lookahead=1.0, wheelbase=0.5)

# Logging for plotting
xs, ys = [], []

# 3. Run the simulation for ~5 seconds
dt = 0.02  # 50 Hz
steps = int(30 / dt)

print("Starting Pure Pursuit + PID validation...")

for step in range(steps):
    # Compute control
    v, delta = controller.compute_control(robot, path)

    # Apply control to robot
    robot.set_control(v, delta)
    robot.update(dt)

    # Log
    rx, ry, rtheta = robot.pose()
    xs.append(rx)
    ys.append(ry)

    if step % 25 == 0:
        print(f"Step {step}: pos=({rx:.2f}, {ry:.2f}), v={robot.v:.2f}, delta={math.degrees(delta):.2f}°")

    time.sleep(dt)

# 4. Plot the result 
try:
    import matplotlib.pyplot as plt
    px, py = zip(*path)

    plt.figure(figsize=(6, 6))
    plt.plot(px, py, 'g--', label="Reference Path")
    plt.plot(xs, ys, 'b-', label="Robot Trajectory")
    plt.scatter(xs[0], ys[0], c='red', label="Start")
    plt.legend()
    plt.axis('equal')
    plt.title("Pure Pursuit + PID Validation")
    plt.show()
except ImportError:
    print("Matplotlib not installed — skipping trajectory plot.")
