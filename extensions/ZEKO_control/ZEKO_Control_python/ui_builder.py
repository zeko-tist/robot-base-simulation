"""UIBuilder – coordinator for the ZEKO Control extension.

Keeps the same public interface as the original (on_menu_callback,
on_timeline_event, on_physics_step, on_stage_event, on_key_press,
on_key_release, cleanup, build_ui) so extension.py needs zero changes.

Domain logic lives in managers/; UI construction lives in panels/.
All isaacsim.core imports happen lazily inside managers so this module
is safe to import at extension startup time.
"""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style

from .managers.console_manager import ConsoleManager
from .managers.connection_manager import ConnectionManager
from .managers.keyboard_manager import KeyboardManager
from .managers.telemetry_manager import TelemetryManager
from .managers.navigation_manager import NavigationManager

from .panels import (
    robot_panel,
    control_panel,
    telemetry_panel,
    console_panel,
    settings_panel,
    navigation_panel,
)


class UIBuilder:

    def __init__(self):
        # Shared state dict – passed by reference into managers.
        self._state = {
            "robot":             None,
            "controller":        None,
            "is_connected":      False,
            "linear_speed":      5.0,
            "angular_speed":     3.0,
            "keyboard_enabled":  True,
            "telemetry_enabled": True,
        }

        # UI label store – panels write here; managers read via getters.
        self._labels: dict = {}

        # Console history list.
        self._console_history: list = []

        # Wrapped UI elements for cleanup.
        self.wrapped_ui_elements = []

        # --- Managers ---
        self._console = ConsoleManager(
            history=self._console_history,
            label_getter=lambda: self._labels.get("console"),
        )

        self._conn = ConnectionManager(
            state=self._state,
            console_manager=self._console,
            label_getter=lambda: self._labels.get("status"),
        )

        self._kb = KeyboardManager(
            state=self._state,
            connection_manager=self._conn,
            console_manager=self._console,
        )

        self._telem = TelemetryManager(
            state=self._state,
            labels={k: (lambda k=k: self._labels.get(k))
                    for k in ("pos_x", "pos_y", "pos_z",
                               "yaw", "left_vel", "right_vel")},
        )

        self._nav = NavigationManager(console_manager=self._console)

    # ── Lifecycle (called by extension.py – interface unchanged) ──────────

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        pass

    def on_physics_step(self, step):
        self._telem.update()

    def on_stage_event(self, event):
        self._conn.disconnect()

    def on_key_press(self, key: str):
        self._kb.on_key_press(key)

    def on_key_release(self, key: str):
        self._kb.on_key_release(key)

    def cleanup(self):
        for elem in self.wrapped_ui_elements:
            if hasattr(elem, "cleanup"):
                elem.cleanup()
        self.wrapped_ui_elements.clear()
        self._labels.clear()
        self._conn.disconnect()

    # ── UI construction ───────────────────────────────────────────────────

    def build_ui(self):
        robot_panel.build(
            labels=self._labels,
            wrapped=self.wrapped_ui_elements,
        )

        control_panel.build(
            on_forward=self._on_forward,
            on_backward=self._on_backward,
            on_left=self._on_left,
            on_right=self._on_right,
            on_stop=self._on_stop,
            wrapped=self.wrapped_ui_elements,
        )

        telemetry_panel.build(
            labels=self._labels,
            wrapped=self.wrapped_ui_elements,
        )

        navigation_panel.build(
            nav_manager=self._nav,
            wrapped=self.wrapped_ui_elements,
        )

        settings_panel.build(
            state=self._state,
            wrapped=self.wrapped_ui_elements,
        )

        console_panel.build(
            labels=self._labels,
            on_clear=self._console.clear,
            wrapped=self.wrapped_ui_elements,
        )

        self._console.log("ZEKO Control ready.")

    # ── Control button callbacks ──────────────────────────────────────────

    def _on_forward(self):
        if self._conn.ensure_connected():
            self._state["controller"].forward()
            self._console.log("Moving forward.")

    def _on_backward(self):
        if self._conn.ensure_connected():
            self._state["controller"].backward()
            self._console.log("Moving backward.")

    def _on_left(self):
        if self._conn.ensure_connected():
            self._state["controller"].left()
            self._console.log("Turning left.")

    def _on_right(self):
        if self._conn.ensure_connected():
            self._state["controller"].right()
            self._console.log("Turning right.")

    def _on_stop(self):
        if self._conn.ensure_connected():
            self._state["controller"].stop()
            self._console.log("Stopped.")
