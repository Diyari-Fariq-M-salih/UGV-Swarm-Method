# Multi-UAV Potential Field Navigation — Project Proposal & TODO

## 📌 Project Overview

This project proposal number 2 aims to recreate and expand the multi-UAV or UGV mission simulator into a full Python application using **PySide6**, **Matplotlib**, and **Potential Field (PF) navigation** instead of RRT/RRT\*.  
The simulator will feature:

- Real-time GUI control
- Obstacle drawing
- Two UAVs with bicycle kinematics
- Attractive–repulsive potential field planner
- Collision avoidance
- Modular and scalable architecture

---

## 🎯 Goals

- Build a clean, maintainable Python project structure.
- Implement a robust PF-based motion controller.
- Provide real-time visualization similar to MATLAB’s GUI.
- Enable future extensions (missions, swarm behaviors, ROS2 integration).

---

## 🧩 System Components

1. **GUI Layer (PySide6)**

   - Buttons: _Start, Stop, Clear, Add Obstacle_
   - Dynamic obstacle placement
   - Planner selection
   - Embedded Matplotlib canvas

2. **Simulation Core**

   - Manages UAV states, time stepping, and collision detection
   - Invokes PF planner
   - Sends commands to controller

3. **Potential Field Planner**

   - Attractive field: goal direction
   - Repulsive field: obstacles + UAVs
   - Gradient descent / force-based motion

4. **Bicycle Model Controller**

   - Converts force vector into steering + velocity commands
   - Handles dynamics constraints

5. **Environment Map**
   - Binary occupancy grid
   - Set/get occupancy
   - Inflation for safety margin
