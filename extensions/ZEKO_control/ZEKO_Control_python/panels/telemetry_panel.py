"""TelemetryPanel – live position, orientation and wheel velocity readouts."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


def build(labels: dict, wrapped: list) -> None:
    """Build the Robot State panel, storing label references in `labels`."""
    frame = CollapsableFrame("Robot State", collapsed=False)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=5):

            ui.Label("Position", style={"color": 0xFFAAAAAA})
            labels["pos_x"] = ui.Label("X  :  0.000 m")
            labels["pos_y"] = ui.Label("Y  :  0.000 m")
            labels["pos_z"] = ui.Label("Z  :  0.000 m")

            ui.Spacer(height=6)
            ui.Label("Orientation", style={"color": 0xFFAAAAAA})
            labels["yaw"] = ui.Label("Yaw  :  0.00°")

            ui.Spacer(height=6)
            ui.Label("Wheel Velocity", style={"color": 0xFFAAAAAA})
            labels["left_vel"]  = ui.Label("Left   :  0.00 rad/s")
            labels["right_vel"] = ui.Label("Right  :  0.00 rad/s")
