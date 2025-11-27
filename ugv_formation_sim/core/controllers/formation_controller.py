import math
import time
from core import formation_shapes

def rotation_matrix(theta):
    """2D rotation matrix."""
    return [
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ]

def apply_rotation(vec, theta):
    """Rotate 2D vector."""
    R = rotation_matrix(theta)
    x = R[0][0]*vec[0] + R[0][1]*vec[1]
    y = R[1][0]*vec[0] + R[1][1]*vec[1]
    return (x, y)


class FormationController:
    def __init__(self, formation_type="triangle", d=1.5, transition_speed=0.7):
        """
        Args:
            formation_type (str): initial formation
            d (float): scale / spacing
            transition_speed (float): how fast transition sigmoid moves (a)
        """
        self.current_offsets = self.load_offsets(formation_type, d)
        self.target_offsets = self.current_offsets[:]  # initial target same as current
        self.transition_active = False

        self.d = d
        self.a = transition_speed
        self.t0 = None  # transition start time

    # Load static offsets for a formation
    def load_offsets(self, formation_type, d):
        if formation_type == "triangle":
            return formation_shapes.triangle(d)
        elif formation_type == "diamond":
            return formation_shapes.diamond(d)
        elif formation_type == "arrow":
            return formation_shapes.arrow(d)
        else:
            raise ValueError(f"Unknown formation: {formation_type}")

    # Initiate a new formation transition
    def set_formation(self, formation_type):
        """User selects new formation — begin smooth transition."""
        self.target_offsets = self.load_offsets(formation_type, self.d)
        self.transition_active = True
        self.t0 = time.time()  # timestamp to anchor transition

    # Smooth interpolation using tanh
    def interpolate_offsets(self):
        """Smoothly morph current δ_i → target δ_i."""
        if not self.transition_active:
            return self.current_offsets

        t = time.time()
        tau = t - self.t0

        sigma = 0.5 * (1 + math.tanh(self.a * (tau - 1)))  
        # shift by 1 sec so it starts gradually

        new_offsets = []
        done = True

        for (cx, cy), (tx, ty) in zip(self.current_offsets, self.target_offsets):
            nx = cx + (tx - cx) * sigma
            ny = cy + (ty - cy) * sigma
            new_offsets.append((nx, ny))

            if abs(nx - tx) > 0.01 or abs(ny - ty) > 0.01:
                done = False

        # If finished: freeze
        if done:
            self.transition_active = False
            self.current_offsets = self.target_offsets[:]
        else:
            self.current_offsets = new_offsets[:]

        return self.current_offsets

    # Compute desired positions for all robots
    def assign_targets(self, leader_pose):
        """
        Args:
            leader_pose (tuple): (x, y, theta)

        Returns:
            list of (x_des, y_des): target positions for each robot
        """
        xL, yL, thetaL = leader_pose

        offsets = self.interpolate_offsets()

        targets = []
        for dx, dy in offsets:
            # Apply rotation
            rx, ry = apply_rotation((dx, dy), thetaL)

            # Translate to leader
            tx = xL + rx
            ty = yL + ry

            targets.append((tx, ty))

        return targets
