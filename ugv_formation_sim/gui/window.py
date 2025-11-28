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

        # Left Control Panel
        control_panel = QVBoxLayout()

        # Buttons
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

        # Leader selection
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

        # Action buttons
        self.btn_plan = QPushButton("Plan Path")
        self.btn_place = QPushButton("Place UGVs")
        self.btn_run = QPushButton("Run Simulation")
        self.btn_reset = QPushButton("Reset All")

        control_panel.addWidget(self.btn_plan)
        control_panel.addWidget(self.btn_place)
        control_panel.addWidget(self.btn_run)
        control_panel.addWidget(self.btn_reset)
        control_panel.addStretch()

        
        # Canvas Setup
        
        fig = Figure()
        self.canvas = FigureCanvas(fig)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_title("Environment")

        main_layout.addLayout(control_panel, 1)
        main_layout.addWidget(self.canvas, 4)

        
        # Internal State
        
        self.mode = None
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.path = []

        self.robots = []
        # Smoothed follower targets
        self.follower_targets = [(0,0), (0,0), (0,0)]
        self.manual_robot_index = 0
        self.spread_counter = 0
        self.is_sim_running = False

        self.controller = PurePursuitController()

        
        # Event Connections
        
        self.btn_add_circle.clicked.connect(self.activate_add_circle)
        self.btn_add_rect.clicked.connect(self.activate_add_rect)
        self.btn_set_start.clicked.connect(self.activate_set_start)
        self.btn_set_goal.clicked.connect(self.activate_set_goal)
        self.btn_plan.clicked.connect(self.plan_path)
        self.btn_place.clicked.connect(self.place_ugvs)
        self.btn_run.clicked.connect(self.run_simulation)
        self.btn_reset.clicked.connect(self.reset_all)

        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)

    
    def smooth_target(self, prev, new, alpha=0.15):
        """Exponential smoothing to prevent sudden formation jumps."""
        px, py = prev
        nx, ny = new
        return (px + alpha * (nx - px),
                py + alpha * (ny - py))


    def obstacle_repulsion(self, robot, influence_radius=2.5, gain=1.3):
        """Returns a repulsive force vector pushing robot away from obstacles."""
        rx, ry, _ = robot.pose()
        fx, fy = 0.0, 0.0

        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                dx, dy = rx - cx, ry - cy
                dist = math.hypot(dx, dy) - r

            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                cx = min(max(rx, x1), x2)
                cy = min(max(ry, y1), y2)
                dx, dy = rx - cx, ry - cy
                dist = math.hypot(dx, dy)

            else:
                continue

            if dist < influence_radius and dist > 0.0001:
                strength = gain / (dist ** 2)
                fx += strength * (dx / dist)
                fy += strength * (dy / dist)

        return fx, fy

    # Interaction Modes
    
    def activate_add_circle(self):
        self.mode = "add_circle"

    def activate_add_rect(self):
        self.mode = "add_rect"

    def activate_set_start(self):
        self.mode = "set_start"

    def activate_set_goal(self):
        self.mode = "set_goal"

    
    # Canvas Click Handler
    
    def on_canvas_click(self, event):
        if event.xdata is None:
            return

        x, y = event.xdata, event.ydata

        # Start point
        if self.mode == "set_start":
            self.start_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'go', markersize=10)
            self.canvas.draw()
            return

        # Goal point
        if self.mode == "set_goal":
            self.goal_point = (x, y)
            self.redraw_scene()
            self.ax.plot(x, y, 'ro', markersize=10)
            self.canvas.draw()
            return

        # Circle obstacle
        if self.mode == "add_circle":
            self.obstacles.append(("circle", x, y, 1.0))
            circ = plt.Circle((x, y), 1.0, color='r', alpha=0.3)
            self.ax.add_patch(circ)
            self.canvas.draw()
            return

        # Rectangle obstacle
        if self.mode == "add_rect":
            w, h = 2.0, 1.5
            self.obstacles.append(("rect", x, y, x + w, y + h))
            rect = plt.Rectangle((x, y), w, h, color='m', alpha=0.3)
            self.ax.add_patch(rect)
            self.canvas.draw()
            return

        # Manual robot placement
        if self.mode == "manual_place":
            if self.manual_robot_index < 3:
                self.robots.append(UGV(x, y, 0))
                self.ax.plot(x, y, 'bs', markersize=10)
                self.canvas.draw()
                self.manual_robot_index += 1

                if self.manual_robot_index == 3:
                    print("All 3 robots placed.")
                    self.mode = None
            else:
                print("Manual placement completed. Reset to place again.")
            return

    
    # Redraw Scene
    
    def redraw_scene(self):
        self.ax.clear()
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_aspect("equal")
        self.ax.set_title("Environment")

        # Obstacles
        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
                self.ax.add_patch(circ)
            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, color='m', alpha=0.3)
                self.ax.add_patch(rect)

        # Start/goal
        if self.start_point:
            self.ax.plot(self.start_point[0], self.start_point[1], 'go', markersize=10)

        if self.goal_point:
            self.ax.plot(self.goal_point[0], self.goal_point[1], 'ro', markersize=10)

    
    # Formation Initialization
    
    def init_in_formation(self):
        sx, sy = self.start_point
        offsets = [(0, 0), (-1, -1), (1, -1)]
        self.robots = [UGV(sx + ox, sy + oy, 0) for ox, oy in offsets]

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

    
    # Path Planning
    
    def plan_path(self):
        if self.start_point is None or self.goal_point is None:
            print("Start/goal missing.")
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

        # Draw path
        xs = [p[0] for p in self.path]
        ys = [p[1] for p in self.path]
        self.ax.plot(xs, ys, 'b-', linewidth=2)
        self.canvas.draw()

    
    # Place UGVs
    
    def place_ugvs(self):
        init_mode = self.init_mode_select.currentText()
        print("Placing UGVs with mode:", init_mode)

        if self.start_point is None:
            print("Start point missing.")
            return

        if init_mode != "Manual Placement" and len(self.path) == 0:
            print("Plan path first.")
            return

        if init_mode == "Start in Formation":
            self.init_in_formation()

        elif init_mode == "Overlapped (Spread Out)":
            self.init_overlapped()

        elif init_mode == "Manual Placement":
            self.init_manual()
            print("Manual placement enabled.")
            return

        # draw robots
        self.redraw_scene()
        for robot in self.robots:
            self.draw_robot(robot)
        self.canvas.draw()

    
    # Reset
    
    def reset_all(self):
        self.start_point = None
        self.goal_point = None
        self.obstacles = []
        self.robots = []
        self.manual_robot_index = 0
        self.path = []
        self.is_sim_running = False

        self.redraw_scene()
        self.canvas.draw()

    
    # Run Simulation
    
    def run_simulation(self):

        if len(self.robots) < 3:
            print("Place UGVs first.")
            return

        if len(self.path) == 0:
            print("Plan path first.")
            return

        print("Simulation started.")
        self.is_sim_running = True

        self.timer = QTimer()
        self.timer.setInterval(20)  # 50 Hz
        self.timer.timeout.connect(self.sim_step)
        self.timer.start()

    
    # Simulation Step
    
    def sim_step(self):
        if not self.is_sim_running:
            return

        dt = 0.02

        leader_idx = self.leader_select.currentIndex()
        leader = self.robots[leader_idx]

        # leader follows path
        v, delta = self.controller.compute_control(leader, self.path)
        leader.set_control(v, delta)
        leader.update(dt)

        # formation tracking
        offsets = [(0,0), (-1,-1), (1,-1)]
        ct, st = math.cos(leader.theta), math.sin(leader.theta)

        rotated_offsets = []
        for ox, oy in offsets:
            rx = ox * ct - oy * st
            ry = ox * st + oy * ct
            rotated_offsets.append((rx, ry))

        for i, robot in enumerate(self.robots):
            if i == leader_idx:
                continue

            # --- Desired formation position (unfiltered) ---
            dx, dy = rotated_offsets[i]
            raw_tx = leader.x + dx
            raw_ty = leader.y + dy

            # --- Smooth the formation target ---
            prev_tx, prev_ty = self.follower_targets[i]
            smooth_tx, smooth_ty = self.smooth_target((prev_tx, prev_ty), (raw_tx, raw_ty))
            self.follower_targets[i] = (smooth_tx, smooth_ty)

            # --- Add obstacle avoidance force ---
            fx, fy = self.obstacle_repulsion(robot)

            final_tx = smooth_tx + fx
            final_ty = smooth_ty + fy

            # --- Drive robot toward final corrected target ---
            follower_path = [(robot.x, robot.y), (final_tx, final_ty)]
            v_f, d_f = self.controller.compute_control(robot, follower_path)
            robot.set_control(v_f, d_f)
            robot.update(dt)


        # draw everything
        self.ax.clear()
        self.ax.set_xlim(0,20)
        self.ax.set_ylim(0,20)
        self.ax.set_aspect("equal")
        self.ax.set_title("Simulation")

        self.draw_obstacles()

        if self.path:
            xs = [p[0] for p in self.path]
            ys = [p[1] for p in self.path]
            self.ax.plot(xs, ys, "b--")

        for robot in self.robots:
            self.draw_robot(robot)

        lx, ly = leader.x, leader.y
        for i, robot in enumerate(self.robots):
            if i != leader_idx:
                self.ax.plot([lx, robot.x], [ly, robot.y], "gray", linestyle="--")

        self.canvas.draw()

    
    # Draw Obstacles
    
    def draw_obstacles(self):
        for obs in self.obstacles:
            if obs[0] == "circle":
                _, cx, cy, r = obs
                circ = plt.Circle((cx, cy), r, color='r', alpha=0.3)
                self.ax.add_patch(circ)
            elif obs[0] == "rect":
                _, x1, y1, x2, y2 = obs
                rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, color='m', alpha=0.3)
                self.ax.add_patch(rect)

    
    # Draw Robot
    
    def draw_robot(self, robot):
        x, y, t = robot.pose()
        size = 0.35

        p1 = (x + size * math.cos(t),     y + size * math.sin(t))
        p2 = (x + size * math.cos(t+2.5), y + size * math.sin(t+2.5))
        p3 = (x + size * math.cos(t-2.5), y + size * math.sin(t-2.5))

        xs = [p1[0], p2[0], p3[0]]
        ys = [p1[1], p2[1], p3[1]]

        self.ax.fill(xs, ys, color="blue")
