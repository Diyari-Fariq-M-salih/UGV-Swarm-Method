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

        
        # Layout Setup
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        # Control Panel
        control_panel = QVBoxLayout()

        self.btn_add_circle = QPushButton("Add Circle Obstacle")
        self.btn_add_rect = QPushButton("Add Rectangle Obstacle")
        self.btn_set_start = QPushButton("Set Start Point")
        self.btn_set_goal = QPushButton("Set Goal Point")

        control_panel.addWidget(self.btn_add_circle)
        control_panel.addWidget(self.btn_add_rect)
        control_panel.addWidget(self.btn_set_start)
        control_panel.addWidget(self.btn_set_goal)

        # Planner
        control_panel.addWidget(QLabel("Motion Planner:"))
        self.planner_select = QComboBox()
        self.planner_select.addItems(["A*", "RRT", "Potential Field"])
        control_panel.addWidget(self.planner_select)

        # Formation
        control_panel.addWidget(QLabel("Formation:"))
        self.formation_select = QComboBox()
        self.formation_select.addItems(["Triangle"])
        control_panel.addWidget(self.formation_select)

        # Leader
        control_panel.addWidget(QLabel("Leader Robot:"))
        self.leader_select = QComboBox()
        self.leader_select.addItems(["UGV 1", "UGV 2", "UGV 3"])
        control_panel.addWidget(self.leader_select)

        # Initialization Mode
        control_panel.addWidget(QLabel("Formation Initialization:"))
        self.init_mode_select = QComboBox()
        self.init_mode_select.addItems([
            "Start in Formation",
            "Overlapped (Spread Out)",
            "Manual Placement"
        ])
        control_panel.addWidget(self.init_mode_select)

        # Action Buttons
        self.btn_place = QPushButton("Place UGVs")
        self.btn_plan = QPushButton("Plan Path")
        self.btn_run = QPushButton("Run Simulation")
        self.btn_reset = QPushButton("Reset All")

        control_panel.addWidget(self.btn_place)
        control_panel.addWidget(self.btn_plan)
        control_panel.addWidget(self.btn_run)
        control_panel.addWidget(self.btn_reset)
        control_panel.addStretch()

        # Canvas
        fig = Figure()
        self.canvas = FigureCanvas(fig)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_title("Environment")

        main_layout.addLayout(control_panel, 1)
        main_layout.addWidget(self.canvas, 4)

        
        # State Variables
        
        self.mode = None
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.path = []

        self.robots = []
        self.manual_robot_index = 0
        self.is_sim_running = False

        self.controller = PurePursuitController()

        # === CONSENSUS Formations Setup ===
        # Triangle formation offsets
        self.rest_offsets = [
            (0, 0),      # Leader
            (-1, -1),    # Robot 2
            (1, -1)      # Robot 3
        ]

        # Rest lengths for consensus graph
        def _dist(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
        self.rest_lengths = {
            (0, 1): _dist(self.rest_offsets[0], self.rest_offsets[1]),
            (0, 2): _dist(self.rest_offsets[0], self.rest_offsets[2]),
            (1, 2): _dist(self.rest_offsets[1], self.rest_offsets[2]),
        }

        # Connections
        self.btn_add_circle.clicked.connect(self.activate_add_circle)
        self.btn_add_rect.clicked.connect(self.activate_add_rect)
        self.btn_set_start.clicked.connect(self.activate_set_start)
        self.btn_set_goal.clicked.connect(self.activate_set_goal)
        self.btn_plan.clicked.connect(self.plan_path)
        self.btn_place.clicked.connect(self.place_ugvs)
        self.btn_reset.clicked.connect(self.reset_all)
        self.btn_run.clicked.connect(self.run_simulation)

        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)

   
    # Interaction Modes
   
    def activate_add_circle(self): self.mode = "add_circle"
    def activate_add_rect(self): self.mode = "add_rect"
    def activate_set_start(self): self.mode = "set_start"
    def activate_set_goal(self): self.mode = "set_goal"

   
    # Canvas Click Handler
   
    def on_canvas_click(self, event):
        if event.xdata is None:
            return

        x, y = float(event.xdata), float(event.ydata)

        if self.mode == "set_start":
            self.start_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'go', markersize=10)
            self.canvas.draw()
            return

        if self.mode == "set_goal":
            self.goal_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'ro', markersize=10)
            self.canvas.draw()
            return

        if self.mode == "add_circle":
            self.obstacles.append(("circle", x, y, 1.0))
            circ = plt.Circle((x, y), 1.0, color='r', alpha=0.3)
            self.ax.add_patch(circ)
            self.canvas.draw()
            return

        if self.mode == "add_rect":
            w, h = 2.0, 1.5
            self.obstacles.append(("rect", x, y, x + w, y + h))
            rect = plt.Rectangle((x, y), w, h, color='m', alpha=0.3)
            self.ax.add_patch(rect)
            self.canvas.draw()
            return

        # Manual UGV placement
        if self.mode == "manual_place":
            if self.manual_robot_index < 3:
                self.robots.append(UGV(x, y, 0))
                self.ax.plot(x, y, 'bs', markersize=10)
                self.canvas.draw()
                self.manual_robot_index += 1

                if self.manual_robot_index == 3:
                    print("All 3 robots placed.")
                    self.mode = None
            return

   
    # Redraw Scene
   
    def redraw_scene(self):
        self.ax.clear()
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_aspect("equal")
        self.ax.set_title("Environment")

        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
                self.ax.add_patch(circ)
            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, color='m', alpha=0.3)
                self.ax.add_patch(rect)

        if self.start_point:
            self.ax.plot(*self.start_point, "go", markersize=10)

        if self.goal_point:
            self.ax.plot(*self.goal_point, "ro", markersize=10)

   
    # Formation Initialization
   
    def init_in_formation(self):
        sx, sy = self.start_point
        self.robots = [UGV(sx + ox, sy + oy, 0)
                       for ox, oy in self.rest_offsets]

    def init_overlapped(self):
        sx, sy = self.start_point
        self.robots = [UGV(sx, sy, 0), UGV(sx, sy, 0), UGV(sx, sy, 0)]

    def init_manual(self):
        print("Manual placement ON.")
        self.manual_robot_index = 0
        self.robots = []
        self.mode = "manual_place"
        self.ax.text(1, 19, "Click 3 robot start positions", color="blue")
        self.canvas.draw()

   
    # Path Planning
   
    def plan_path(self):
        if not self.start_point or not self.goal_point:
            print("Start/Goal missing.")
            return

        self.redraw_scene()

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

        if not self.path:
            print("No path found.")
            return

        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]
        self.ax.plot(xs, ys, "b-", linewidth=2)
        self.canvas.draw()

   
    # UGV Placement
   
    def place_ugvs(self):
        init_mode = self.init_mode_select.currentText()

        if self.start_point is None:
            print("Set start point first.")
            return

        if init_mode != "Manual Placement" and len(self.path) == 0:
            print("Plan path first.")
            return

        if init_mode == "Start in Formation":
            self.init_in_formation()

        elif init_mode == "Overlapped (Spread Out)":
            self.init_overlapped()

        else:
            self.init_manual()
            return

        self.redraw_scene()
        for r in self.robots:
            self.draw_robot(r)
        self.canvas.draw()

   
    # Reset All
   
    def reset_all(self):
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.robots = []
        self.path = []
        self.manual_robot_index = 0
        self.redraw_scene()
        self.canvas.draw()

   
    # Run Simulation
   
    def run_simulation(self):
        if len(self.robots) != 3:
            print("Place 3 robots first.")
            return

        if len(self.path) == 0:
            print("Plan path first.")
            return

        self.is_sim_running = True

        self.timer = QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.sim_step)
        self.timer.start()

   
    # Obstacle Repulsion (Perpendicular)
   
    def obstacle_repulsion(self, robot, R=2.5, gain=1.5):
        rx, ry, _ = robot.pose()
        fx, fy = 0.0, 0.0

        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, rr = obs
                dx = rx - cx
                dy = ry - cy
                dist = math.hypot(dx, dy) - rr

            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                cx = min(max(rx, x1), x2)
                cy = min(max(ry, y1), y2)
                dx = rx - cx
                dy = ry - cy
                dist = math.hypot(dx, dy)

            else:
                continue

            if dist < R and dist > 1e-5:
                mag = gain / (dist * dist)
                fx += mag * dx / (dist + 1e-6)
                fy += mag * dy / (dist + 1e-6)

        return fx, fy
    
    def draw_obstacles(self):
        """Draw all static obstacles on the simulation canvas."""
        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
                self.ax.add_patch(circ)

            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, color='m', alpha=0.3)
                self.ax.add_patch(rect)

   
    # Consensus-Based Simulation Step
   
    def sim_step(self):
        if not self.is_sim_running:
            return

        dt = 0.02

        # --- 1. Leader motion ---
        leader_idx = self.leader_select.currentIndex()
        leader = self.robots[leader_idx]

        v, delta = self.controller.compute_control(leader, self.path)
        leader.set_control(v, delta)
        leader.update(dt)

        # --- 2. Desired formation positions ---
        offsets = self.rest_offsets
        ct = math.cos(leader.theta)
        st = math.sin(leader.theta)

        desired_positions = []
        for ox, oy in offsets:
            rx = ox * ct - oy * st
            ry = ox * st + oy * ct
            desired_positions.append((leader.x + rx, leader.y + ry))

        # --- 3. Consensus update ---
        k_c = 0.8   # consensus
        k_f = 1.4   # formation
        k_o = 0.25  # obstacle

        positions = [(r.x, r.y) for r in self.robots]

        for i, robot in enumerate(self.robots):

            if i == leader_idx:
                continue

            xi, yi = positions[i]
            px_des, py_des = desired_positions[i]

            # Consensus toward neighbors
            dx_c = sum((xj - xi) for j,(xj,yj) in enumerate(positions) if j!=i)
            dy_c = sum((yj - yi) for j,(xj,yj) in enumerate(positions) if j!=i)
            vx_c = k_c * dx_c
            vy_c = k_c * dy_c

            # Formation pull
            vx_f = k_f * (px_des - xi)
            vy_f = k_f * (py_des - yi)

            # Obstacle avoidance (perpendicular)
            Ox, Oy = self.obstacle_repulsion(robot)
            Fx, Fy = (px_des - xi, py_des - yi)
            L = math.hypot(Fx, Fy)

            if L > 1e-6:
                Fx /= L; Fy /= L
                dot = Ox*Fx + Oy*Fy
                Ox -= dot*Fx
                Oy -= dot*Fy

            vx_o = k_o * Ox
            vy_o = k_o * Oy

            # Combined consensus velocity
            vx = vx_c + vx_f + vx_o
            vy = vy_c + vy_f + vy_o

            tx = xi + vx
            ty = yi + vy

            follower_path = [(xi, yi), (tx, ty)]
            v_f, d_f = self.controller.compute_control(robot, follower_path)
            robot.set_control(v_f, d_f)
            robot.update(dt)

        # --- 4. Draw Simulation ---
        self.ax.clear()
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_aspect("equal")
        self.ax.set_title("Simulation")

        self.draw_obstacles()

        if self.path:
            xs = [p[0] for p in self.path]
            ys = [p[1] for p in self.path]
            self.ax.plot(xs, ys, "b--")

        for robot in self.robots:
            self.draw_robot(robot)

        # Draw formation edges (consensus graph)
        for (i, j), _ in self.rest_lengths.items():
            xi, yi, _ = self.robots[i].pose()
            xj, yj, _ = self.robots[j].pose()
            self.ax.plot([xi, xj], [yi, yj], "gray", linestyle="--")

        self.canvas.draw()

   
    # Robot Rendering
   
    def draw_robot(self, robot):
        x, y, t = robot.pose()
        size = 0.4

        p1 = (x + size * math.cos(t), y + size * math.sin(t))
        p2 = (x + size * math.cos(t + 2.3), y + size * math.sin(t + 2.3))
        p3 = (x + size * math.cos(t - 2.3), y + size * math.sin(t - 2.3))

        self.ax.fill([p1[0], p2[0], p3[0]],
                     [p1[1], p2[1], p3[1]],
                     color="blue")
