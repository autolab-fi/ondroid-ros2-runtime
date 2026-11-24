"""Timed line-following evidence from observations owned by the caller."""
import math


class LineFollowProgress:
    def __init__(self, duration):
        self.duration = float(duration)
        if not math.isfinite(self.duration) or not 0 < self.duration <= 55:
            raise ValueError('Line-follow duration must be between 0 and 55 seconds')
        self.started = self.updated = None
        self.initial = self.anchor = None
        self.distance = self.displacement = self.forward_seconds = 0.0

    def observe(self, now, x, y, linear):
        if self.initial is None:
            self.initial = self.anchor = (x, y)
        self.displacement = max(self.displacement, math.hypot(x-self.initial[0], y-self.initial[1]))
        step = math.hypot(x-self.anchor[0], y-self.anchor[1])
        # Accumulate measured travel above camera jitter, not every noisy frame.
        if step >= .015:
            self.distance += step
            self.anchor = (x, y)
        if linear > .005:
            if self.started is None:
                self.started = now
            if self.updated is not None:
                self.forward_seconds += min(.5, max(0., now-self.updated))
        self.updated = now

    def ready(self, now):
        return (self.started is not None and now-self.started >= self.duration-.2
                and self.forward_seconds >= .8*self.duration
                and self.distance >= .45 and self.displacement >= .25)

    def metrics(self):
        return dict(duration_seconds=self.duration, travel_m=round(self.distance, 3),
                    max_displacement_m=round(self.displacement, 3),
                    forward_seconds=round(self.forward_seconds, 3))
