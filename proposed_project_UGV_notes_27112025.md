# UGV Time-Varying Formation Control Simulation — Project Notes

## 1. Project Architecture (Python-Only Simulation Environment)

- No ROS, no Gazebo, deadline too close — lightweight standalone Python system.
- Modular structure for clarity and expansion:

```
ugv_formation_sim/
│
├── core/
│   ├── ugv_model.py
│   ├── controllers/
│   │       ├── formation_controller.py
│   │       ├── trajectory_tracking.py
│   ├── planners/
│   │       ├── astar.py
│   │       ├── rrt.py
│   │       ├── potential_field.py
│   ├── formation_shapes.py
│   ├── simulator.py
│
├── gui/
│   ├── window.py
│   ├── obstacle_editor.py
│   ├── path_planner_widget.py
│   ├── formation_widget.py
│
├── assets/ to be determined
│
├── logs/ for report making
│
└── main.py
```

**Key goals:**

- 2D visualization
- obstacle editing
- selectable motion planners
- formation management
- real-time simulation

---

## 2. UGV Dynamics — Realistic Ackermann/Unicycle Model

We simulate 3 UGVs with the standard nonholonomic model:

$$
\dot x = v \cos \theta
$$

$$
\dot y = v \sin \theta
$$

$$
\dot \theta = \frac{v}{L} \tan(\delta)
$$

Where:

- **v** = linear velocity (control input)
- **δ** = steering angle (control input)
- **θ** = orientation
- **L** = wheelbase (~0.5m)

**Reasons for using this model:**

- Matches real UGV constraints
- Compatible with Pure Pursuit tracking
- Easy to simulate in real-time

---

## 3. Formation Representation

Formations are defined via **relative offsets** from a leader robot:

$$
p_i^{des}(t) = p_{leader}(t) + R(\theta) , \delta_i(t)
$$

Where:

- **R(θ)** is rotation matrix
- **δᵢ(t)** defines formation shape

### Example Formations (3 robots)

#### **Triangle**

```
R1: ( 0,  0)
R2: (-d, -d)
R3: ( d, -d)
```

#### **Diamond** (3‑robot adaptation)

```
R1: ( 0,  0)
R2: (-d, -d)
R3: ( d, -d)
```

#### **Arrow**

```
R1: ( 0,  0)
R2: (-d, -d)
R3: ( d, -d)
```

---

## 4. Controller Architecture

Because UGVs are nonholonomic, we use a **hierarchical control structure**:

### **(A) Formation Control Layer**

Computes desired goal pose for each robot:

$$
p_i^{des} = p_{leader} + R(\theta)\delta_i(t)
$$

### **(B) Trajectory Tracking Layer — Pure Pursuit**

Selected steering controller:

$$
\delta = \arctan\left(\frac{2L \sin(\alpha)}{d_{lookahead}}\right)
$$

Where:

- **α** = heading error to target point
- **dₗₒₒₖₐₕₑₐd** = lookahead distance
- **L** = wheelbase

**Why Pure Pursuit?**

- Works well with Ackermann vehicles
- Robust and simple
- Good for formation tracking

---

## 5. Motion Planners

Three planners will be implemented:

### **A\***

- Grid‑based
- Deterministic, optimal
- Good for structured maps

### **RRT / RRT\***

- Sampling‑based
- Handles complex environments
- Produces smooth paths (RRT\*)

### **Potential Field Method**

- Good for visualization
- Not globally optimal but intuitive
- Shows repulsive + attractive forces

The GUI will allow selecting.

---

## 6. GUI Design using PyQt5 + Matplotlib Canvas

### **Main Features**

- 2D top‑down simulation canvas
- Live animation of the robots and path

### **Left Control Panel**

- Add/delete obstacles (circle or rectangle)
- Set start and goal points
- Select planner: A\*, RRT, Potential Field
- Select formation: triangle / diamond / arrow
- Start, pause, reset simulation

### **Bottom Panel**

- Status output and logs
- Planner runtime, path length, tracking error

--

## 7. Time‑Varying Formation Support

Support smooth shape transitions:

$$
\delta_i(t) = \delta_i^0 + \delta_i^1 \tanh(a(t - t_0))
$$

Used for:

- Switching triangle → arrow
- Obstacle‑driven formation changes
- Manual GUI‑triggered changes

Inspired by formation transition functions used in the referenced paper.

---

## 8. Report Support

The system will automatically generate logs for:

- robot trajectories
- control inputs
- formation errors
- planner performance

We will later produce:

### Report Sections

- Introduction / Motivation
- Literature Review
- UGV Kinematic Model
- Formation Definitions
- Controller Derivations
- Planner Algorithms
- System Architecture
- Simulation Results
- Discussion / Limitations
- Conclusion

These notes serve as the foundation for the report and the coding phase. so please pay attention :3
