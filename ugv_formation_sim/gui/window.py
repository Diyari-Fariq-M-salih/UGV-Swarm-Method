import sys
import math
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QLabel, QComboBox
)
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from core.ugv_model import UGV
from core.controllers.trajectory_tracking import PurePursuitController


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UGV Formation Simulator")
        self.setGeometry(100, 50, 1200, 700)

        # ======================
        # Layout Setup
        # ======================
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        # Left Control Panel
        control_panel = QVBoxLayout()

        # ========== Buttons ==========
        self.btn_add_circle = QPushButton("Add Circle Obstacle")
        self.btn_add_rect = QPushButton("Add Rectangle Obstacle")
        self.btn_set_start = QPushButton("Set Start Point")
        self.btn_set_goal = QPushButton("Set Goal Point")

        control_panel.addWidget(self.btn_add_circle)
        control_panel.addWidget(self.btn_add_rect)
        control_panel.addWidget(self.btn_set_start)
        control_panel.addWidget(self.btn_set_goal)

        # ========== Planner selection ==========
        control_panel.addWidget(QLabel("Motion Planner:"))
        self.planner_select = QComboBox()
        self.planner_select.addItems(["A*", "RRT", "Potential Field"])
        control_panel.addWidget(self.planner_select)

        # ========== Formation selection ==========
        control_panel.addWidget(QLabel("Formation:"))
        self.formation_select = QComboBox()
        self.formation_select.addItems(["Triangle", "Diamond", "Arrow"])
        control_panel.addWidget(self.formation_select)

        # ========== Leader selection ==========
        control_panel.addWidget(QLabel("Leader Robot:"))
        self.leader_select = QComboBox()
        self.leader_select.addItems(["UGV 1", "UGV 2", "UGV 3"])
        control_panel.addWidget(self.leader_select)

        # ========== Initialization Mode ==========
        control_panel.addWidget(QLabel("Formation Initialization:"))
        self.init_mode_select = QComboBox()
        self.init_mode_select.addItems([
            "Start in Formation",
            "Overlapped (Spread Out)",
            "Manual Placement"
        ])
        control_panel.addWidget(self.init_mode_select)

        # ========== Action buttons ==========
        self.btn_plan = QPushButton("Plan Path")
        self.btn_run = QPushButton("Run Simulation")
        self.btn_reset = QPushButton("Reset All")

        control_panel.addWidget(self.btn_plan)
        control_panel.addWidget(self.btn_run)
        control_panel.addWidget(self.btn_reset)
        control_panel.addStretch()

        # ======================
        # Canvas (Matplotlib)
        # ======================
        fig = Figure()
        self.canvas = FigureCanvas(fig)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_title("Environment")

        main_layout.addLayout(control_panel, 1)
        main_layout.addWidget(self.canvas, 4)

        # ======================
        # State Variables
        # ======================
        self.mode = None                 # for clicking modes
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.path = []

        self.robots = []
        self.manual_robot_index = 0
        self.spread_counter = 0
        self.is_sim_running = False

        self.controller = PurePursuitController()

        # ======================
        # Connections
        # ======================
        self.btn_add_circle.clicked.connect(self.activate_add_circle)
        self.btn_add_rect.clicked.connect(self.activate_add_rect)
        self.btn_set_start.clicked.connect(self.activate_set_start)
        self.btn_set_goal.clicked.connect(self.activate_set_goal)
        self.btn_plan.clicked.connect(self.plan_path)
        self.btn_reset.clicked.connect(self.reset_all)
        self.btn_run.clicked.connect(self.run_simulation)

        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)

    # ==============================
    # Interaction Modes
    # ==============================
    def activate_add_circle(self):
        self.mode = "add_circle"

    def activate_add_rect(self):
        self.mode = "add_rect"

    def activate_set_start(self):
        self.mode = "set_start"

    def activate_set_goal(self):
        self.mode = "set_goal"

    # ==============================
    # Canvas Click Handler
    # ==============================
    def on_canvas_click(self, event):
        if event.xdata is None:
            return

        x, y = event.xdata, event.ydata

        # Start point (ONLY ONE!)
        if self.mode == "set_start":
            self.start_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'go', markersize=10)
            self.canvas.draw()
            return

        # Goal point (ONLY ONE!)
        if self.mode == "set_goal":
            self.goal_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'ro', markersize=10)
            self.canvas.draw()
            return

        # Add circle
        if self.mode == "add_circle":
            self.obstacles.append(("circle", x, y, 1.0))
            circ = plt.Circle((x, y), 1.0, color='r', alpha=0.3)
            self.ax.add_patch(circ)
            self.canvas.draw()
            return

        # Add rectangle
        if self.mode == "add_rect":
            w, h = 2.0, 1.5
            self.obstacles.append(("rect", x, y, x + w, y + h))
            rect = plt.Rectangle((x, y), w, h, color='m', alpha=0.3)
            self.ax.add_patch(rect)
            self.canvas.draw()
            return

        # Manual robot placement
        # -----------------------------------------------------------
        # Manual placement of robots (Step 4)
        # -----------------------------------------------------------
        elif self.mode == "manual_place":
            # Allow EXACTLY 3 robots
            if self.manual_robot_index < 3:
                x, y = event.xdata, event.ydata

                # Create robot
                self.robots.append(UGV(x, y, 0))

                # Draw robot spawn marker
                self.ax.plot(x, y, 'bs', markersize=10)
                self.canvas.draw()

                self.manual_robot_index += 1
                print(f"Robot {self.manual_robot_index} placed at ({x:.2f}, {y:.2f})")

                # If done → exit placement mode
                if self.manual_robot_index == 3:
                    print("All 3 robots placed.")
                    self.mode = None

            # If somehow clicks happen AFTER placing 3 robots
            else:
                print("Manual placement completed. Reset to place again.")
            return


    # ==============================
    # Redraw all obstacles / start / goal
    # ==============================
    def redraw_scene(self):
        self.ax.clear()
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_aspect("equal")
        self.ax.set_title("Environment")

        # Draw obstacles
        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
                self.ax.add_patch(circ)
            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, color='m', alpha=0.3)
                self.ax.add_patch(rect)

        # Draw start/goal
        if self.start_point is not None:
            self.ax.plot(self.start_point[0], self.start_point[1], "go", markersize=10)

        if self.goal_point is not None:
            self.ax.plot(self.goal_point[0], self.goal_point[1], "ro", markersize=10)

    # ==============================
    # Formation Initialization
    # ==============================
    def init_in_formation(self):
        sx, sy = self.start_point
        offsets = [(0, 0), (-1, -1), (1, -1)]
        self.robots = [
            UGV(sx + ox, sy + oy, 0) for ox, oy in offsets
        ]

    def init_overlapped(self):
        sx, sy = self.start_point
        self.robots = [UGV(sx, sy, 0), UGV(sx, sy, 0), UGV(sx, sy, 0)]
        self.spread_counter = 0

    def init_manual(self):
        print("Manual placement ON.")
        self.manual_robot_index = 0
        self.robots = []
        self.mode = "manual_place"
        self.ax.text(1, 19, "Click 3 robot start positions", color="blue")
        self.canvas.draw()

    # ==============================
    # Path Planning
    # ==============================
    def plan_path(self):
        if self.start_point is None or self.goal_point is None:
            print("Start/goal missing.")
            return

        self.redraw_scene()

        # Choose planner
        name = self.planner_select.currentText()

        if name == "A*":
            from core.planners.astar import AStarPlanner
            planner = AStarPlanner(grid_resolution=0.3)
        elif name == "RRT":
            from core.planners.rrt import RRTPlanner
            planner = RRTPlanner()
        else:
            from core.planners.potential_field import PotentialFieldPlanner
            planner = PotentialFieldPlanner()

        self.path = planner.plan(self.start_point, self.goal_point, self.obstacles)

        if len(self.path) == 0:
            print("No path found.")
            return

        # Draw path
        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]
        self.ax.plot(xs, ys, "b-", linewidth=2)
        self.canvas.draw()

    # ==============================
    # Reset All
    # ==============================
    def reset_all(self):
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.robots = []
        self.path = []
        self.manual_robot_index = 0

        self.redraw_scene()
        self.canvas.draw()

    # ==============================
    # Simulation (added in Step 5 later)
    # ==============================
    def run_simulation(self):
        # User must plan a path first
        if self.start_point is None or self.goal_point is None or len(self.path) == 0:
            print("Plan a path first.")
            return

        mode = self.init_mode_select.currentText()

        # -----------------------------------------
        # Start in Formation
        # -----------------------------------------
        if mode == "Start in Formation":
            self.init_in_formation()
            print("Robots initialized in formation.")
            print(self.robots)
            return   # Step 5 will later start the timer

        # -----------------------------------------
        # Overlapped
        # -----------------------------------------
        elif mode == "Overlapped (Spread Out)":
            self.init_overlapped()
            print("Robots initialized overlapped (will spread).")
            print(self.robots)
            return   # Step 5 adds motion later

        # -----------------------------------------
        # Manual Placement
        # -----------------------------------------
        elif mode == "Manual Placement":
            self.init_manual()
            # This prints: "Manual placement ON.\nClick 3 robot start positions"
            print("Waiting for user to place 3 robots...")
            return

