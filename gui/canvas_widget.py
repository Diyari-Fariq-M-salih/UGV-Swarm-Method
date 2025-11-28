import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure


class CanvasWidget(QWidget):
    """
    Matplotlib canvas inside a Qt widget.
    Handles all drawing of the simulation.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # optional: live images for efficiency
        self.im = None

    def draw_environment(self, env):
        """Draw occupancy grid."""
        grid = env.grid

        self.ax.clear()
        self.ax.imshow(
            grid,
            cmap="Greys",
            origin="lower",
            extent=[0, env.width, 0, env.height],
            vmin=0,
            vmax=1
        )
        self.ax.set_aspect('equal')

    def draw_uav(self, state, color="blue", L=0.5):
        """Draw a UAV using rectangular footprint."""
        x, y, theta, _, _ = state

        # UAV rectangle
        length = L * 1.5
        width = L * 1.0

        corners_local = np.array([
            [ length/2,  width/2],
            [-length/2,  width/2],
            [-length/2, -width/2],
            [ length/2, -width/2],
            [ length/2,  width/2],
        ])

        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)],
        ])
        rotated = (R @ corners_local.T).T

        xs = rotated[:, 0] + x
        ys = rotated[:, 1] + y

        self.ax.plot(xs, ys, color=color, linewidth=2)

    def draw_goal(self, goal, color="red", marker="x"):
        self.ax.plot(goal[0], goal[1], marker=marker,
                     markersize=12, color=color)

    def refresh(self):
        self.canvas.draw()
