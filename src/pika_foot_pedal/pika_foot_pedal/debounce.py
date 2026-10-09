"""Physical, level-based switch handling without ROS dependencies."""


class PedalDebouncer:
    def __init__(self, debounce_ms=30.0):
        if debounce_ms < 0:
            raise ValueError('debounce_ms must be non-negative')
        self.debounce_ns = int(debounce_ms * 1_000_000)
        self.pressed = False
        self.armed = False
        self.active_press = False
        self.raw = False
        self.changed_ns = 0

    def initialize(self, held, now_ns):
        # A held pedal at process startup must be released before it can START.
        self.pressed = bool(held)
        self.raw = bool(held)
        self.armed = not held
        self.active_press = False
        self.changed_ns = now_ns

    def event(self, value, now_ns):
        if value not in (0, 1):
            return
        new_raw = value == 1
        if new_raw != self.raw:
            self.raw = new_raw
            self.changed_ns = now_ns

    def tick(self, now_ns):
        """Return 'press', 'release', or None after a stable edge."""
        if self.raw == self.pressed or now_ns - self.changed_ns < self.debounce_ns:
            return None
        self.pressed = self.raw
        if not self.pressed:
            was_active = self.active_press
            self.active_press = False
            self.armed = True
            return 'release' if was_active else None
        if self.armed:
            self.armed = False
            self.active_press = True
            return 'press'
        return None

    def disconnect(self):
        was_pressed = self.pressed or self.raw
        self.pressed = False
        self.raw = False
        self.armed = False
        self.active_press = False
        return was_pressed
