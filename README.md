# 🚁 Multi-UAV Path Planning & Local Navigation Simulator

_A modular simulation framework for A\*, RRT, Potential Fields, and
Elastic Band._

## 👥 Authors

- **Mohammed-salih Diyari\*** - **Chaabeni Ilyes\***

## 🌟 Overview

This project provides a clean and intuitive simulation environment for
**multi-UAV navigation**, supporting global planners like **A\*** and
**RRT**, as well as local planners such as **Potential Fields** and
**Elastic Band (EBand)**.

Everything is wrapped inside an interactive GUI, making it easy to
visualize motion, debug planning behavior, and experiment with dynamic
obstacles.

The simulator is designed to be **modular**, **extendable**, and
**educational**, while still supporting advanced research-style
algorithms.

## 🚀 Features

### ✔️ Global Path Planning

- **A\*** (8-direction grid search, optimal and efficient)
- **RRT** (sampling-based, supports smoothing and tree visualization)

### ✔️ Local Planning

- **Potential Field (PF)** --- reactive, fully online
- **Elastic Band (EBand)** --- bubble-based smoothing that adapts to
  obstacles

### ✔️ Multi-Agent Simulation

- Two UAVs simulated at once
- Independent planning + mutual avoidance
- Each UAV may use a different planner

### ✔️ Dynamic Replanning

- Automatically reacts when obstacles block the path
- Smart waypoint validation
- Only replans when needed (efficient)

### ✔️ Dynamic Obstacles

- Horizontal and vertical moving blocks
- Bounce behavior
- Used by global + local planners

### ✔️ Real-Time Visualization

- Live UAV rendering
- Goals, paths, RRT trees
- Optional Elastic Band bubble visualization
- Dynamic & static obstacles
- GIF recording for demos

## 🧱 Project Structure

    /core
        simulation.py        → Multi-UAV simulation engine
        environment.py       → Maps, inflation, occupancy handling
        uav_controller.py    → Pure pursuit & UAV motion model
        formation_controller → Formation control modules

    /planners
        a_star.py            → A* global planner
        rrt.py               → RRT planner
        potential_field.py   → Reactive potential field
        eband.py             → Elastic Band smoothing
        test_eband.py        → EBand standalone test

    /gui
        window.py            → GUI controller (Qt)
        canvas_widget.py     → Visualization (matplotlib)

    main.py                  → Startup script
    logs/                    → Saved GIF animations

## ⚙️ How It Works

1.  **Global Planner**: A\* or RRT computes an obstacle-aware global
    path.\
2.  **Elastic Band (optional)**: The path is smoothed using "bubbles"
    that deform around obstacles.\
3.  **Local Motion**: Pure pursuit transforms the smoothed path into
    velocity commands.\
4.  **Simulation**: UAVs avoid obstacles, react to the environment, and
    navigate to their goals.

# 🛠️ Setup & Usage Guide

## ⚙️ Installation & Environment Setup

This project does **not** require Anaconda specifically.  
You may use **any Python virtual environment**, such as:

- `venv` (recommended)
- `virtualenv`
- Conda / Mamba (optional)
- Poetry (optional)

### 1. Create a virtual environment

Using `venv`:

```bash
python -m venv uav_env
```

Activate it:

- **Windows**
  ```bash
  uav_env\Scripts\activate
  ```
- **Linux / macOS**
  ```bash
  source uav_env/bin/activate
  ```

### 2. Install dependencies

```bash
pip install numpy matplotlib PySide6 scipy
```

(Optional for GIF export):

```bash
pip install imageio
```

---

## ▶️ Running the Simulation

```bash
python main.py
```

Use the GUI to:

- Select the planner (PF, A\*, RRT, A\*+EBand, RRT+EBand)
- Draw static obstacles
- Add dynamic obstacles
- Start, stop, and reset the simulation
- Record animations

---

## 🙌 Acknowledgments

Thanks to the robotics and open-source communities whose work inspired
this simulator.
