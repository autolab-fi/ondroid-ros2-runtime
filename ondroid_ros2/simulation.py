"""Deterministic differential-drive fixture in the existing arena coordinate frame.

The server owns grading and the fixed-step model. The browser displays telemetry,
so losing a tab never leaves an unbounded simulation or robot command running.
"""
import math
import time


class Simulation:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.x, self.y, self.yaw = .40, -.185, 0.
        self.linear = self.angular = 0.
        self.updated = clock()
        self.sequence = 0
        self.command_at = None

    def sample(self):
        now = self.clock()
        remaining = min(now-self.updated, .25)
        while remaining > 1e-9:
            dt = min(.01, remaining)
            if self.command_at is not None and now-self.command_at <= .5:
                self.x += self.linear * math.cos(self.yaw + self.angular*dt/2) * dt
                self.y += self.linear * math.sin(self.yaw + self.angular*dt/2) * dt
                self.yaw = math.atan2(math.sin(self.yaw+self.angular*dt), math.cos(self.yaw+self.angular*dt))
            remaining -= dt
        self.updated = now
        self.sequence += 1
        return dict(x=self.x, y=self.y, yaw=self.yaw, sequence=self.sequence, received_monotonic=now)

    def drive(self, linear, angular, pose):
        self.sample()
        self.linear, self.angular = linear, angular
        self.command_at = self.clock()

    def stop(self):
        self.linear = self.angular = 0.
