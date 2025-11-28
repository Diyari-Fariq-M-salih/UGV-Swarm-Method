import numpy as np
from PySide6.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QComboBox
)
from PySide6.QtCore import QTimer
from gui.canvas_widget import CanvasWidget

class MainWindow(QWidget):
    def __init__(self, env, sim):
        super().__init__()

        self.env = env
        self.sim = sim

        self.setWindowTitle("Multi-UAV PF Simulator (Smooth FPS)")
        self.resize(900, 700)

        # Canvas
        self.canvas = CanvasWidget(self)

        # Controls
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.clear_btn = QPushButton("Clear Obstacles")
        self.add_obs_btn = QPushButton("Add Obstacle Mode")

        self.horiz_btn = QPushButton("Add Horizontal Moving Obstacle")
        self.vert_btn = QPushButton("Add Vertical Moving Obstacle")

        self.planner_label = QLabel("Planner:")
        self.planner_dropdown = QComboBox()
        self.planner_dropdown.addItems(["Potential Field"])

        self.status_label = QLabel("Status: Idle")

        self.add_obs_btn.setCheckable(True)

        # Layout
        top = QHBoxLayout()
        top.addWidget(self.start_btn)
        top.addWidget(self.stop_btn)
        top.addWidget(self.clear_btn)
        top.addWidget(self.add_obs_btn)
        top.addWidget(self.planner_label)
        top.addWidget(self.planner_dropdown)

        mid = QHBoxLayout()
        mid.addWidget(self.horiz_btn)
        mid.addWidget(self.vert_btn)

        layout = QVBoxLayout()
        layout.addLayout(top)
        layout.addLayout(mid)
        layout.addWidget(self.canvas)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

        # Events
        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(self.on_stop)
        self.clear_btn.clicked.connect(self.on_clear)
        self.add_obs_btn.clicked.connect(self.toggle_add_obstacle)
        self.horiz_btn.clicked.connect(self.add_horizontal)
        self.vert_btn.clicked.connect(self.add_vertical)

        self.canvas.canvas.mpl_connect("button_press_event", self.on_click)

        # DOUBLE TIMER SYSTEM

        # Physics timer (simulation)
        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self.on_sim_step)
        self.sim_timer.start(int(self.sim.dt * 1000))   # dt=0.1 → 10 Hz

        # Render timer (graphics refresh)
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_canvas)
        self.render_timer.start(16)                     # ~60 FPS

        self.add_mode = False

    # Button/Control Handlers
    def on_start(self):
        self.sim.start()
        self.status_label.setText("Status: Running")

    def on_stop(self):
        self.sim.stop()
        self.status_label.setText("Status: Stopped")

    def toggle_add_obstacle(self):
        self.add_mode = self.add_obs_btn.isChecked()
        self.add_obs_btn.setText(
            "Click to Add Obstacle" if self.add_mode else "Add Obstacle Mode"
        )

    def add_horizontal(self):
        self.sim.add_horizontal_obstacle(y=15, speed=1.5)

    def add_vertical(self):
        self.sim.add_vertical_obstacle(x=10, speed=1.5)

    def on_click(self, event):
        if not self.add_mode:
            return
        if event.xdata is None:
            return
        self.env.set_occupancy_world(event.xdata, event.ydata, 1)
        self.update_canvas()

    def on_clear(self):
        self.env.clear()
        self.sim.reset()
        self.update_canvas()

    # Physics Timer Callback (10 Hz)
    def on_sim_step(self):
        if self.sim.running:
            self.sim.step()

    # Render Timer Callback (~60 FPS)
    def update_canvas(self):
        self.canvas.draw_environment(self.env)

        u1, u2, g1, g2, dyn_list = self.sim.get_states()

        self.canvas.draw_uav(u1, "blue", L=self.sim.controller.L)
        self.canvas.draw_uav(u2, "red",  L=self.sim.controller.L)

        self.canvas.draw_goal(g1, "blue", marker="x")
        self.canvas.draw_goal(g2, "red",  marker="o")

        for obs in dyn_list:
            self.canvas.draw_dynamic_obstacle(obs)

        self.canvas.refresh()
