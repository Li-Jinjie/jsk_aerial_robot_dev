"""
 Created by li-jinjie on 2025/8/17.
"""
import math


class PIDController:
    """Simple PID controller with anti-windup and output clamp."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, i_limit: float = float("inf"), out_limit: float = float("inf")):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_limit = float(i_limit)
        self.out_limit = float(out_limit)
        self.integral = 0.0
        self.prev_err = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def step(self, err: float, dt: float) -> float:
        if dt <= 0.0:
            # Fallback: pure P if no dt
            return max(-self.out_limit, min(self.out_limit, self.kp * err))
        # P
        p = self.kp * err
        # I with anti-windup
        self.integral += err * dt
        if abs(self.integral) > self.i_limit:
            self.integral = math.copysign(self.i_limit, self.integral)
        i = self.ki * self.integral
        # D
        d = 0.0
        if self.prev_err is not None:
            d = self.kd * (err - self.prev_err) / dt
        self.prev_err = err
        u = p + i + d
        # Clamp
        if abs(u) > self.out_limit:
            u = math.copysign(self.out_limit, u)
        return u
