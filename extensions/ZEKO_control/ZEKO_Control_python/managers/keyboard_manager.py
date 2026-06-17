"""KeyboardManager – W/A/S/D/Space → robot commands, key-repeat suppressed."""

MOVEMENT_KEYS = frozenset({"W", "A", "S", "D"})

PRESS_MAP = {
    "W":     ("Moving forward.",  "forward"),
    "S":     ("Moving backward.", "backward"),
    "A":     ("Turning left.",    "left"),
    "D":     ("Turning right.",   "right"),
    "SPACE": ("Stopped.",         "stop"),
}


class KeyboardManager:

    def __init__(self, state: dict, connection_manager, console_manager):
        self._s = state
        self._conn = connection_manager
        self._console = console_manager
        self._active = set()

    def on_key_press(self, key: str) -> None:
        key = key.upper()
        if not self._s.get("keyboard_enabled", True):
            return
        if key in self._active:
            return
        self._active.add(key)

        if key not in PRESS_MAP:
            return
        if not self._conn.ensure_connected():
            return

        label, method = PRESS_MAP[key]
        getattr(self._s["controller"], method)()
        self._console.log(label)

    def on_key_release(self, key: str) -> None:
        key = key.upper()
        self._active.discard(key)
        if not self._s.get("keyboard_enabled", True):
            return
        if key not in MOVEMENT_KEYS:
            return
        if self._s["controller"] is None:
            return
        self._s["controller"].stop()
        self._console.log("Stopped.")
