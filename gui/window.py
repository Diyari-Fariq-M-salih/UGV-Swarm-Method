import numpy as np
from PySide6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QComboBox
from PySide6.QtCore import QTimer
from gui.canvas_widget import CanvasWidget

from planners.potential_field import PotentialFieldPlanner
from planners.a_star import AStarPlanner
from planners.rrt import RRTPlanner

import imageio
import copy


class MainWindow(QWidget):
    def __init__(self, env, sim):
        super().__init__()

        self.env = env
        self.sim = sim

        self.setWindowTitle("Multi-UAV Simulator with PF / A* / RRT + Visualization")
        self.resize(900, 700)

        self.canvas = CanvasWidget(self)

        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.clear_btn = QPushButton("Clear Obstacles")
        self.add_obs_btn = QPushButton("Add Obstacle Mode")
        self.gif_btn = QPushButton("Record GIF")

        self.horiz_btn = QPushButton("Add Horizontal Moving Obstacle")
        self.vert_btn = QPushButton("Add Vertical Moving Obstacle")

        self.planner_label = QLabel("Planner:")
        self.planner_dropdown = QComboBox()
        self.planner_dropdown.addItems(["Potential Field", "A*", "RRT"])

        self.status_label = QLabel("Status: Idle")

        self.add_obs_btn.setCheckable(True)

        top = QHBoxLayout()
        top.addWidget(self.start_btn)
        top.addWidget(self.stop_btn)
        top.addWidget(self.clear_btn)
        top.addWidget(self.add_obs_btn)
        top.addWidget(self.planner_label)
        top.addWidget(self.planner_dropdown)
        top.addWidget(self.gif_btn)

        mid = QHBoxLayout()
        mid.addWidget(self.horiz_btn)
        mid.addWidget(self.vert_btn)

        layout = QVBoxLayout()
        layout.addLayout(top)
        layout.addLayout(mid)
        layout.addWidget(self.canvas)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(self.on_stop)
        self.clear_btn.clicked.connect(self.on_clear)
        self.add_obs_btn.clicked.connect(self.toggle_add_obstacle)
        self.horiz_btn.clicked.connect(self.add_horizontal)
        self.vert_btn.clicked.connect(self.add_vertical)
        self.gif_btn.clicked.connect(self.toggle_gif_record)

        self.canvas.canvas.mpl_connect("button_press_event", self.on_click)

        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self.on_sim_step)
        self.sim_timer.start(int(self.sim.dt * 1000))

        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_canvas)
        self.render_timer.start(16)

        self.recording = False
        self.frames = []

        self.add_mode = False

    # ----------------------------------------------------------------------
    # BUTTONS
    # ----------------------------------------------------------------------

    def on_start(self):
        choice = self.planner_dropdown.currentText()

        if choice == "Potential Field":
            planner = PotentialFieldPlanner()
        elif choice == "A*":
            planner = AStarPlanner()
        elif choice == "RRT":
            planner = RRTPlanner()

        self.sim.planner1 = copy.deepcopy(planner)
        self.sim.planner2 = copy.deepcopy(planner)

        self.sim.compute_paths()
        self.sim.start()
        self.status_label.setText(f"Running {choice}")

    def on_stop(self):
        self.sim.stop()
        self.status_label.setText("Stopped")

    def toggle_add_obstacle(self):
        self.add_mode = self.add_obs_btn.isChecked()
        self.add_obs_btn.setText(
            "Click to Add Obstacle" if self.add_mode else "Add Obstacle Mode"
        )

    def add_horizontal(self):
        pass

    def add_vertical(self):
        pass

    def on_click(self, event):
        if self.add_mode and event.xdata is not None:
            self.env.set_occupancy_world(event.xdata, event.ydata, 1)
            self.update_canvas()

    def on_clear(self):
        self.env.clear()
        self.sim.reset()
        self.update_canvas()

    # ----------------------------------------------------------------------
    # GIF
    # ----------------------------------------------------------------------

    def toggle_gif_record(self):
        if not self.recording:
            self.frames = []
            self.recording = True
            self.gif_btn.setText("Stop & Save GIF")
        else:
            self.recording = False
            self.gif_btn.setText("Record GIF")
            try:
                imageio.mimsave("simulation.gif", self.frames, fps=15)
                self.status_label.setText("Saved simulation.gif")
            except Exception as e:
                self.status_label.setText(f"GIF save failed: {e}")

    # ----------------------------------------------------------------------
    # TIMERS
    # ----------------------------------------------------------------------

    def on_sim_step(self):
        if self.sim.running:
            self.sim.step()

    # ----------------------------------------------------------------------
    # DRAW CANVAS
    # ----------------------------------------------------------------------

    def update_canvas(self):
        self.canvas.draw_environment(self.env)

        u1, u2, g1, g2, dyn = self.sim.get_states()

        self.canvas.draw_uav(u1, "blue", L=self.sim.controller.L)
        self.canvas.draw_uav(u2, "red", L=self.sim.controller.L)

        self.canvas.draw_goal(g1, "blue", marker="x")
        self.canvas.draw_goal(g2, "red", marker="o")

        # BOTH UAV PLANNERS
        p1 = self.sim.planner1
        p2 = self.sim.planner2

        from planners.a_star import AStarPlanner
        from planners.rrt import RRTPlanner

        # UAV1 path (yellow) + tree (cyan)
        if isinstance(p1, AStarPlanner):
            self.canvas.draw_path(p1.path, "yellow")
        elif isinstance(p1, RRTPlanner):
            self.canvas.draw_rrt_tree(p1.tree_edges, "cyan")
            self.canvas.draw_path(p1.path, "yellow")

        # UAV2 path (orange) + tree (magenta)
        if isinstance(p2, AStarPlanner):
            self.canvas.draw_path(p2.path, "orange")
        elif isinstance(p2, RRTPlanner):
            self.canvas.draw_rrt_tree(p2.tree_edges, "magenta")
            self.canvas.draw_path(p2.path, "orange")

        # Metrics (show UAV1)
        try:
            if isinstance(p1, AStarPlanner):
                txt = f"A* | Time {p1.plan_time:.1f}ms | Length {p1.path_length:.2f} | Expanded {p1.expanded_count}"
            else:
                txt = f"RRT | Time {p1.plan_time:.1f}ms | Length {p1.path_length:.2f} | Nodes {p1.node_count}"
            self.status_label.setText(txt)
        except:
            pass

        self.canvas.refresh()

        if self.recording:
            buf = self.canvas.fig.canvas.buffer_rgba()
            self.frames.append(np.array(buf))
