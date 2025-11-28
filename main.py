import sys
import numpy as np

from PySide6.QtWidgets import QApplication

from core.environment import OccupancyGrid
from planners.potential_field import PotentialFieldPlanner
from core.uav_controller import UAVController
from core.simulation import MultiUAVSimulation
from gui.window import MainWindow


def main():
    # 1. Initialize QApplication
    app = QApplication(sys.argv)

    # 2. Create Environment (30x30 like MATLAB version)
    env = OccupancyGrid(width=30, height=30, resolution=1.0)

    # Add borders as obstacles (optional)
    for x in range(30):
        env.set_occupancy_grid(x, 0, 1)
        env.set_occupancy_grid(x, 29, 1)
    for y in range(30):
        env.set_occupancy_grid(0, y, 1)
        env.set_occupancy_grid(29, y, 1)

    # 3. Create Potential Field Planner
    planner = PotentialFieldPlanner(
        k_att=1.2,
        k_rep=2.5,
        k_rep_uav=1.0,
        obstacle_influence=3.0,
        uav_influence=3.0,
        force_limit=4.0
    )

    # 4. Create UAV Controller (bicycle model)
    controller = UAVController(
        wheelbase=0.5,
        dt=0.1,
        max_vel=2.0,
        max_accel=1.0,
        max_steer=np.pi/6,
        max_steer_rate=np.pi/4
    )

    # 5. Create Simulation Core
    goal1 = np.array([27.5, 27.5])
    goal2 = np.array([27.5, 2.5])

    sim = MultiUAVSimulation(
        env=env,
        planner=planner,
        controller=controller,
        goal1=goal1,
        goal2=goal2,
        dt=0.1,
        safety_distance=2.0
    )

    # 6. Create and Show Main Window
    window = MainWindow(env, sim)
    window.show()

    # 7. Start Qt Event Loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
