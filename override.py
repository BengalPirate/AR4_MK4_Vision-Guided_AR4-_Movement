"""Operator override: repeat the same refused action three times to force it.

The third consecutive attempt of an action that our software refused (within
WINDOW_S seconds) is sent anyway, skipping our checks (path simulation, pen-angle
guard, wrist limits, detection/workspace checks). The firmware's own limits still
apply. Every forced move is announced and logged.
"""
import time

TRIES = 3
WINDOW_S = 20.0


class Override:
    def __init__(self):
        self.key, self.count, self.t = None, 0, 0.0

    def refused(self, key):
        """Record a refusal; return how many more tries force it."""
        now = time.monotonic()
        if key != self.key or now - self.t > WINDOW_S:
            self.key, self.count = key, 0
        self.count += 1
        self.t = now
        return max(0, TRIES - 1 - self.count)

    def armed(self, key):
        """True if this attempt should be forced."""
        return key == self.key and self.count >= TRIES - 1 \
            and time.monotonic() - self.t <= WINDOW_S

    def reset(self):
        self.key, self.count = None, 0
