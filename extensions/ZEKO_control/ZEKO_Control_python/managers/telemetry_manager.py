"""TelemetryManager – reads robot state each physics step and updates labels."""

import math


class TelemetryManager:

    def __init__(self, state: dict, labels: dict):
        """
        Args:
            state: shared dict with keys: robot, controller, telemetry_enabled.
            labels: dict of label-getter callables keyed by name.
        """
        self._s = state
        self._labels = labels

    def update(self) -> None:
        if not self._s.get("telemetry_enabled", True):
            return
        if self._s["robot"] is None or self._s["controller"] is None:
            return

        try:
            position, orientation = self._s["robot"].get_world_poses()
            joint_state = self._s["robot"].get_joints_state()
            self._update_position(position)
            self._update_orientation(orientation)
            self._update_wheels(joint_state)
        except Exception as exc:
            print(f"[ZEKO] Telemetry error: {exc}")

    def _get(self, key):
        fn = self._labels.get(key)
        return fn() if fn else None

    def _update_position(self, position) -> None:
        for key, idx, axis in [("pos_x", 0, "X"), ("pos_y", 1, "Y"), ("pos_z", 2, "Z")]:
            lbl = self._get(key)
            if lbl:
                lbl.text = f"{axis}  :  {position[0][idx]:.3f} m"

    def _update_orientation(self, orientation) -> None:
        lbl = self._get("yaw")
        if lbl is None:
            return
        qw, qx, qy, qz = (orientation[0][i] for i in range(4))
        yaw = math.degrees(math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        ))
        lbl.text = f"Yaw  :  {yaw:.2f}°"

    def _update_wheels(self, joint_state) -> None:
        ctrl = self._s["controller"]
        if ctrl is None:
            return
        lv = joint_state.velocities[0, ctrl.left_idx]
        rv = joint_state.velocities[0, ctrl.right_idx]
        for key, val, side in [("left_vel", lv, "Left"), ("right_vel", rv, "Right")]:
            lbl = self._get(key)
            if lbl:
                lbl.text = f"{side}  :  {val:.2f} rad/s"
