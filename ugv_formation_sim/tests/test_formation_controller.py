import time
from core.controllers.formation_controller import FormationController

# ---------------------------
# Initialize formation controller
# ---------------------------
fc = FormationController(formation_type="triangle", d=1.5)

# Leader pose (x, y, theta)
leader_pose = (0.0, 0.0, 0.0)

print("Initial Triangle Formation:")
targets = fc.assign_targets(leader_pose)
for i, t in enumerate(targets):
    print(f"Robot {i+1}: {t}")

# ---------------------------
# Trigger transition → arrow
# ---------------------------
print("\nTransitioning to ARROW formation...")
fc.set_formation("arrow")

# Print frames for 3 seconds at 20 Hz
for step in range(60):
    targets = fc.assign_targets(leader_pose)

    print(f"\nStep {step}")
    for i, t in enumerate(targets):
        print(f"Robot {i+1}: {t}")

    time.sleep(0.05)  # 20 Hz
