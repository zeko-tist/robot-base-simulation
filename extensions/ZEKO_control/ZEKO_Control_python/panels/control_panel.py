"""ControlPanel – simple vertical button list matching the original layout."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


def build(on_forward, on_backward, on_left, on_right, on_stop,
          wrapped: list) -> None:
    frame = CollapsableFrame("Manual Control", collapsed=False)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=5):

            forward  = Button("Forward",  "FORWARD",  on_click_fn=on_forward)
            left     = Button("Left",     "LEFT",     on_click_fn=on_left)
            stop     = Button("Stop",     "STOP",     on_click_fn=on_stop)
            right    = Button("Right",    "RIGHT",    on_click_fn=on_right)
            backward = Button("Backward", "BACKWARD", on_click_fn=on_backward)

            wrapped.extend([forward, left, stop, right, backward])
