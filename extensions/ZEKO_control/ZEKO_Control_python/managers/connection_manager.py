"""ConnectionManager – robot discovery and articulation initialisation.

All isaacsim.core imports are LAZY (inside _connect()) so this module is
safe to import at extension load time before Isaac Sim subsystems are ready.
"""

import numpy as np


class ConnectionManager:

    def __init__(self, state: dict, console_manager, label_getter):
        """
        Args:
            state: shared dict with keys: robot, controller, is_connected.
            console_manager: ConsoleManager instance.
            label_getter: callable returning the current status ui.Label.
        """
        self._s = state
        self._console = console_manager
        self._label_getter = label_getter

    def ensure_connected(self) -> bool:
        if self._s["is_connected"] and self._s["controller"] is not None:
            return True
        return self._connect()

    def disconnect(self) -> None:
        self._s["robot"] = None
        self._s["controller"] = None
        self._s["is_connected"] = False
        self._set_status("🔴 Disconnected")

    def _set_status(self, text: str) -> None:
        lbl = self._label_getter()
        if lbl is not None:
            lbl.text = text

    def _connect(self) -> bool:
        # lazy imports – only safe when simulation is running
        import omni.usd
        from isaacsim.core.prims import Articulation
        from ..controller import ZekoController
        from ..constants import ROBOT_PRIM

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._console.log("No stage open.")
            self._set_status("🔴 No stage")
            return False

        prim = stage.GetPrimAtPath(ROBOT_PRIM)
        if not prim.IsValid():
            self._console.log(f"Robot '{ROBOT_PRIM}' not found.")
            self._set_status("🔴 Not found")
            return False

        try:
            self._set_status("🟡 Connecting…")

            robot = Articulation(prim_paths_expr=ROBOT_PRIM, name="zeko")
            robot.initialize()

            kps = np.zeros((1, robot.num_dof))
            kds = np.full((1, robot.num_dof), 1e5)
            robot.set_gains(kps=kps, kds=kds)

            controller = ZekoController(robot)

            self._s["robot"]        = robot
            self._s["controller"]   = controller
            self._s["is_connected"] = True

            self._set_status("🟢 Connected")
            self._console.log("Robot connected.")
            return True

        except Exception as exc:
            self._s["is_connected"] = False
            self._set_status("🔴 Error")
            self._console.log("Connection failed.")
            print(f"[ZEKO] Connection error: {exc}")
            return False
