"""NavigationPanel – placeholder UI for future waypoint navigation."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style

from ..managers.navigation_manager import DESTINATIONS


def build(nav_manager, wrapped: list) -> None:
    """Build the Navigation panel."""
    frame = CollapsableFrame("Navigation", collapsed=True)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=6):

            ui.Label("Destination", style={"color": 0xFFAAAAAA, "font_size": 12})
            combo = ui.ComboBox(0, *DESTINATIONS)

            def on_dest_changed(model, _item):
                idx = model.get_item_value_model().get_value_as_int()
                nav_manager.set_destination(DESTINATIONS[idx])

            combo.model.add_item_changed_fn(on_dest_changed)

            ui.Spacer(height=4)
            with ui.HStack(spacing=6):
                go_btn = Button("GO", "GO", on_click_fn=nav_manager.go)
                cancel_btn = Button("Cancel", "CANCEL", on_click_fn=nav_manager.cancel)
                wrapped.extend([go_btn, cancel_btn])

            ui.Spacer(height=4)
            ui.Label("⚠  Autonomous navigation not yet implemented.",
                     style={"font_size": 11, "color": 0xFF888800},
                     word_wrap=True)
