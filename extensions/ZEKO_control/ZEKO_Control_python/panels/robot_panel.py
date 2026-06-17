"""RobotPanel – connection status only."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


def build(labels: dict, wrapped: list) -> None:
    frame = CollapsableFrame("Robot", collapsed=False)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=5):
            ui.Label("Connection Status")
            labels["status"] = ui.Label("🟡 Waiting...")
