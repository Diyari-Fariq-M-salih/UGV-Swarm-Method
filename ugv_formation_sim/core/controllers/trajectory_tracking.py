import math
import time

class PID:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = output_limits
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        current_time = time.time()

        if self.prev_time is None:
            dt = 0.01
        else:
            dt = max(current_time - self.prev_time, 1e-6)

        self.prev_time = current_time

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Saturation
        low, high = self.output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)

        return output


class PurePursuitController:
    def __init__(self, lookahead=0.5, wheelbase=0.5):
        self.lookahead = lookahead
        self.L = wheelbase

        # PID for speed control
        self.pid_speed = PID(
            kp=1.5,
            ki=0.4,
            kd=0.2,
            output_limits=(0, 2.0)   # speed limits (m/s)
        )

    # Find target lookahead point on the path
    def find_lookahead_point(self, robot, path):
        rx, ry, _ = robot.pose()

        # Step 1: find closest point index
        closest_idx = 0
        closest_dist = float('inf')

        for i, (px, py) in enumerate(path):
            d = math.hypot(px - rx, py - ry)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # Step 2: lookahead search
        for j in range(closest_idx, len(path)):
            px, py = path[j]
            if math.hypot(px - rx, py - ry) >= self.lookahead:
                return (px, py)

        # Step 3: default to final waypoint
        return path[-1]

    # Compute steering + speed
    def compute_control(self, robot, path):
        rx, ry, rtheta = robot.pose()
        target = self.find_lookahead_point(robot, path)
        tx, ty = target

        # Compute angle to target
        alpha = math.atan2(ty - ry, tx - rx) - rtheta

        # Normalize
        alpha = (alpha + math.pi) % (2*math.pi) - math.pi

        # Pure pursuit steering
        delta = math.atan2(2 * self.L * math.sin(alpha), self.lookahead)

        # Reference speed depends on turning
        v_ref = max(0.6, 2.0 * (1 - abs(delta)))

        # Current speed
        v_current = robot.v

        # PID computes new v
        v = self.pid_speed.compute(v_ref, v_current)

        return v, delta
