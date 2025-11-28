# UGV Formation Control Experiment — Final Summary & Conclusions

## 1. Overview

This experiment explored formation control for **three Ackermann-steered UGVs** operating in a 2D environment with:

- Constrained forward-only motion
- Steering-based heading control
- Path planning (A\*, RRT, Potential Fields)
- Obstacle placement & avoidance
- Formation logic (triangle formation)
- Leader–follower tracking
- Pure Pursuit for trajectory tracking
- A fully interactive PyQt5 GUI

Throughout the experiment, multiple control strategies were attempted:

- Direct geometric follower offsets
- Virtual springs
- Obstacle repulsion
- Consensus formation control

---

## 2. Key Findings

### 2.1 Non-Holonomic Constraints Are the Main Difficulty

Ackermann vehicles **cannot move sideways**, which means:

- Followers cannot directly correct lateral errors.
- Sharp turns by the leader cause followers to drift.
- Obstacle avoidance pushes robots sideways—something they cannot execute efficiently.
- Formation tends to stretch, collapse, or distort under disturbances.

These limitations make stable tight formation control **inherently challenging**.

---

### 2.2 Spring-Based Approaches Are Unstable

Virtual spring networks resulted in:

- Oscillation
- Shape collapse
- High sensitivity to obstacle forces
- Difficult parameter tuning

Springs behave like physics systems—good for animation, not ideal for formation stability.

---

### 2.3 Consensus Control Works Better but Still Struggles

Consensus-based control improved:

- Group cohesion
- Smoothness
- Formation rigidity

However, followers still struggled during:

- Sharp leader turns
- Strong obstacle influence
- Fast leader motion

The root cause remained the same: **non-holonomic movement limits**.

---

## 3. Technical Conclusion

**Ackermann UGVs + Real-time Formation Control + Obstacles = Hard Problem.**

The combination of:

- Forward-only motion
- Limited steering angle
- Non-linear kinematics
- Complex obstacle interactions
- Tight formation requirements

makes this a problem typically solved using:

- **Model Predictive Control (MPC)**
- **Feedback linearization**
- **Advanced nonlinear control**
- **Virtual structure + kinematic transformations**

This exceeds the scope of simple controllers (pure pursuit, springs, consensus).

---

## 4. Recommended Next Steps

### 4.1 Switch to Holonomic UGVs (Strongly Recommended)

A holonomic robot (mecanum wheel, omniwheel, point-mass model) can:

- Move directly sideways
- Correct errors instantly
- Maintain rigid formations
- Avoid obstacles smoothly
- Track consensus perfectly

This will allow clean, elegant formation behavior and clearer experiments.

---

### 4.2 Build a Two-Layer Control Architecture

Keep formation control **holonomic**, then transform motion into Ackermann commands later:

1. **Holonomic Layer**  
   Generates “desired velocity”:

   ```
   vx, vy, ω
   ```

2. **Ackermann Conversion Layer**  
   Converts holonomic commands to:
   ```
   v, δ
   ```

This is a standard approach in multi-robot systems.

---

### 4.3 Try Virtual Structure or Barrier Functions

These methods provide:

- Rigorous formation guarantees
- Smooth shape transitions
- Safe obstacle avoidance

Useful for next experiments.

---

## 5. Final Thoughts

This project successfully demonstrated:

- Multi-robot coordination
- GUI-based environment interaction
- Obstacle-aware planning
- Path following
- Formation behavior under constraints

But it also highlighted how **non-holonomic constraints dominate system behavior**, making simple formation controllers insufficient.

---

## 6. Closing Statement

This experiment was valuable because it revealed _why_ formation control with Ackermann robots is challenging.  
Next experiments will be more productive with holonomic motion models, allowing us to focus on formation algorithms rather than struggling against kinematic limitations.
