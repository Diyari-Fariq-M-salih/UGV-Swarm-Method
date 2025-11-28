from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, 
    QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QComboBox
)
from PyQt5.QtCore import Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import sys
import matplotlib.pyplot as plt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UGV Formation Simulator")
        self.setGeometry(100, 50, 1200, 700)

        # Layout structure: Left panel (controls) + Right (canvas)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Left Control Panel
        control_panel = QVBoxLayout()

        # Add obstacle buttons
        self.btn_add_circle = QPushButton("Add Circle Obstacle")
        self.btn_add_rect = QPushButton("Add Rectangle Obstacle")
        self.btn_set_start = QPushButton("Set Start Point")
        self.btn_set_goal = QPushButton("Set Goal Point")

        control_panel.addWidget(self.btn_add_circle)
        control_panel.addWidget(self.btn_add_rect)
        control_panel.addWidget(self.btn_set_start)
        control_panel.addWidget(self.btn_set_goal)

        # Planner selection
        control_panel.addWidget(QLabel("Motion Planner:"))
        self.planner_select = QComboBox()
        self.planner_select.addItems(["A*", "RRT", "Potential Field"])
        control_panel.addWidget(self.planner_select)

        # Formation selection
        control_panel.addWidget(QLabel("Formation:"))
        self.formation_select = QComboBox()
        self.formation_select.addItems(["Triangle", "Diamond", "Arrow"])
        control_panel.addWidget(self.formation_select)

        # Action buttons
        self.btn_plan = QPushButton("Plan Path")
        self.btn_run = QPushButton("Run Simulation")
        self.btn_reset = QPushButton("Reset All")

        control_panel.addWidget(self.btn_plan)
        control_panel.addWidget(self.btn_run)
        control_panel.addWidget(self.btn_reset)

        control_panel.addStretch()

        # Right Panel: Matplotlib Canvas
        fig = Figure()
        self.canvas = FigureCanvas(fig)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.set_title("Environment")

        # Add panels to main layout
        main_layout.addLayout(control_panel, stretch=1)
        main_layout.addWidget(self.canvas, stretch=4)

        # State variables
        self.mode = None
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.path = []

        # Connect events
        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)

        # Connect button signals
        self.btn_add_circle.clicked.connect(self.activate_add_circle)
        self.btn_add_rect.clicked.connect(self.activate_add_rect)
        self.btn_set_start.clicked.connect(self.activate_set_start)
        self.btn_set_goal.clicked.connect(self.activate_set_goal)
        self.btn_plan.clicked.connect(self.plan_path)
        self.btn_reset.clicked.connect(self.reset_all)

    # Interaction Modes
    def activate_add_circle(self):
        self.mode = "add_circle"

    def activate_add_rect(self):
        self.mode = "add_rect"

    def activate_set_start(self):
        self.mode = "set_start"

    def activate_set_goal(self):
        self.mode = "set_goal"

    # Mouse event on canvas
    def on_canvas_click(self, event):
        if event.xdata is None:
            return

        x, y = event.xdata, event.ydata

        if self.mode == "add_circle":
            self.obstacles.append(("circle", x, y, 0.5))
            circ = plt.Circle((x, y), 0.5, color='r', alpha=0.3)
            self.ax.add_patch(circ)

        elif self.mode == "add_rect":
            w, h = 1.0, 1.0
            self.obstacles.append(("rect", x, y, x+w, y+h))
            rect = plt.Rectangle((x, y), w, h, color='m', alpha=0.3)
            self.ax.add_patch(rect)

        elif self.mode == "set_start":
            self.start_point = (x, y)
            self.ax.plot(x, y, 'go', markersize=10)

        elif self.mode == "set_goal":
            self.goal_point = (x, y)
            self.ax.plot(x, y, 'ro', markersize=10)

        self.canvas.draw()

    # PATH PLANNING
    def plan_path(self):
        if self.start_point is None or self.goal_point is None:
            return

        planner_name = self.planner_select.currentText()

        if planner_name == "A*":
            from core.planners.astar import AStarPlanner
            planner = AStarPlanner(grid_resolution=0.3)

        elif planner_name == "RRT":
            from core.planners.rrt import RRTPlanner
            planner = RRTPlanner()

        else:
            from core.planners.potential_field import PotentialFieldPlanner
            planner = PotentialFieldPlanner()

        self.path = planner.plan(self.start_point, self.goal_point, self.obstacles)

        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]

        self.ax.plot(xs, ys, 'b-', linewidth=2)
        self.canvas.draw()

    # RESET BUTTON
    def reset_all(self):
        self.ax.clear()
        self.ax.set_title("Environment")
        self.ax.set_aspect('equal')

        self.obstacles = []
        self.path = []
        self.start_point = None
        self.goal_point = None
        self.canvas.draw()

    # Run the App
    def run(self):
        self.show()
