import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle

class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def draw_environment(self, env):
        self.ax.clear()

        self.ax.imshow(
            env.get_final_grid(),
            cmap="Greys",
            origin="lower",
            extent=[0, env.width, 0, env.height],
            vmin=0,
            vmax=1,
            interpolation='nearest'
        )

        self.ax.set_xlim(0, env.width)
        self.ax.set_ylim(0, env.height)
        self.ax.set_aspect("equal", "box")

    def draw_path(self, path, color="yellow"):
        if len(path) > 1:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            self.ax.plot(xs, ys, color=color, linewidth=2)

    def draw_rrt_tree(self, edges, color="cyan"):
        for a, b in edges:
            self.ax.plot([a[0], b[0]], [a[1], b[1]], color=color, linewidth=0.6)

    def draw_uav(self, state, color="blue", L=0.5):
        x, y, theta, _, _ = state

        length = L * 1.5
        width = L * 1.0

        pts = np.array([
            [ length/2,  width/2],
            [-length/2,  width/2],
            [-length/2, -width/2],
            [ length/2, -width/2],
        ])

        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        rotated = (R @ pts.T).T
        rotated[:, 0] += x
        rotated[:, 1] += y

        poly = Polygon(rotated, closed=True, color=color)
        self.ax.add_patch(poly)

    def draw_goal(self, goal, color="red", marker="x"):
        self.ax.plot(goal[0], goal[1], marker=marker, markersize=12, color=color)

    # NEW FUNCTION ADDED
    def draw_dynamic_obstacles(self, dyn_list):
        for obs in dyn_list:
            x, y = obs.pos
            s = obs.size
            rect = Rectangle((x - s/2, y - s/2), s, s, color="black")
            self.ax.add_patch(rect)

    def refresh(self):
        self.canvas.draw()
