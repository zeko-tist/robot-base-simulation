"""ConsolePanel – rolling timestamped console with clear button."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


def build(labels: dict, on_clear, wrapped: list) -> None:
    """Build the Console panel."""
    frame = CollapsableFrame("Console", collapsed=False)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=5):

            labels["console"] = ui.Label(
                "", word_wrap=True,
                style={"font_size": 12, "color": 0xFFCCCCCC},
            )

            ui.Spacer(height=4)
            clear_btn = Button("Clear Console", "CLEAR", on_click_fn=on_clear)
            wrapped.append(clear_btn)
