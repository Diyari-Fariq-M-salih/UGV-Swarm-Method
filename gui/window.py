import numpy as np
from PySide6.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QComboBox
)
from PySide6.QtCore import QTimer, Qt

from gui.canvas_widget import CanvasWidget


class MainWindow(QWidget):
    def __init__(self, env, sim):
        super().__init__()

        self.env = env
        self.sim = sim

        self.setWindowTitle("Multi-UAV PF Simulator")
        self.resize(900, 700)

        # Canvas
        self.canvas = CanvasWidget(self)

        # Buttons
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.clear_btn = QPushButton("Clear Obstacles")
        self.add_obs_btn = QPushButton("Add Obstacle Mode")

        # Planner dropdown
        self.planner_label = QLabel("Planner:")
        self.planner_dropdown = QComboBox()
        self.planner_dropdown.addItems(["Potential Field"])

        # Status
        self.status_label = QLabel("Status: Idle")

        # Layout
        top = QHBoxLayout()
        top.addWidget(self.start_btn)
        top.addWidget(self.stop_btn)
        top.addWidget(self.clear_btn)
        top.addWidget(self.add_obs_btn)
        top.addWidget(self.planner_label)
        top.addWidget(self.planner_dropdown)
        top.addStretch(1)

        layout = QVBoxLayout()
        layout.addLayout(top)
        layout.addWidget(self.canvas)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

        # Event connections
        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(self.on_stop)
        self.clear_btn.clicked.connect(self.on_clear)
        self.add_obs_btn.setCheckable(True)
        self.add_obs_btn.clicked.connect(self.toggle_add_obstacle_mode)

        # Mouse events → obstacle painting
        self.canvas.canvas.mpl_connect("button_press_event", self.on_mouse_click)

        # Timer Loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(self.sim.dt * 1000))  # ms

        self.add_mode = False

    # Button Actions
    def on_start(self):
        self.sim.start()
        self.status_label.setText("Status: Running")

    def on_stop(self):
        self.sim.stop()
        self.status_label.setText("Status: Stopped")

    def on_clear(self):
        self.env.clear()
        self.sim.reset()
        self.update_canvas()

    def toggle_add_obstacle_mode(self):
        self.add_mode = self.add_obs_btn.isChecked()
        self.add_obs_btn.setText(
            "Click to Add Obstacle" if self.add_mode else "Add Obstacle Mode"
        )

    # Obstacle Painting
    def on_mouse_click(self, event):
        if not self.add_mode:
            return

        if event.xdata is None or event.ydata is None:
            return

        # Paint 1x1 cell at click
        x, y = event.xdata, event.ydata
        self.env.set_occupancy_world(x, y, 1)
        self.update_canvas()

    # Simulation Timer
    def on_timer(self):
        if not self.sim.running:
            return

        self.sim.step()
        self.update_canvas()

    # Drawing
    def update_canvas(self):
        self.canvas.draw_environment(self.env)

        u1, u2, g1, g2 = self.sim.get_states()
        self.canvas.draw_uav(u1, color="blue", L=self.sim.controller.L)
        self.canvas.draw_uav(u2, color="red",  L=self.sim.controller.L)

        self.canvas.draw_goal(g1, color="blue", marker="x")
        self.canvas.draw_goal(g2, color="red",  marker="o")

        self.canvas.refresh()
