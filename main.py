import sys
import numpy as np
from PySide6.QtWidgets import QApplication

from core.environment import OccupancyGrid
from planners.potential_field import PotentialFieldPlanner
from core.uav_controller import UAVController
from core.simulation import MultiUAVSimulation
from gui.window import MainWindow

def main():
    app = QApplication(sys.argv)

    env = OccupancyGrid(width=30, height=30, resolution=1.0)

    controller = UAVController(dt=0.1)  # physics: 10 Hz
    planner = PotentialFieldPlanner()

    sim = MultiUAVSimulation(
        env=env,
        planner=planner,
        controller=controller,
        goal1=np.array([27.5, 27.5]),
        goal2=np.array([27.5, 2.5]),
        dt=0.1,
        safety_distance=2.0
    )

    window = MainWindow(env, sim)
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
