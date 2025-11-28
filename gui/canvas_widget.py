import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def draw_environment(self, env):
        self.ax.clear()
        self.ax.imshow(
            env.grid,
            cmap="Greys",
            origin="lower",
            extent=[0, env.width, 0, env.height],
            vmin=0, vmax=1
        )
        self.ax.set_aspect('equal')

    def draw_uav(self, state, color="blue", L=0.5):
        x, y, theta, _, _ = state

        length = L * 1.5
        width = L * 1.0
        local = np.array([
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
        pts = (R @ local.T).T
        xs = pts[:, 0] + x
        ys = pts[:, 1] + y
        self.ax.plot(xs, ys, color=color, linewidth=2)

    def draw_goal(self, goal, color="red", marker="x"):
        self.ax.plot(goal[0], goal[1], marker=marker, markersize=12, color=color)

    def draw_dynamic_obstacle(self, obs):
        self.ax.plot(obs.pos[0], obs.pos[1], "ks", markersize=10)

    def refresh(self):
        self.canvas.draw()
