"""SettingsPanel – speed sliders and feature toggles."""

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


def build(state: dict, wrapped: list) -> None:
    """Build the Settings panel. Changes apply immediately via `state`."""
    frame = CollapsableFrame("Settings", collapsed=True)
    wrapped.append(frame)

    with frame:
        with ui.VStack(style=get_style(), spacing=6):

            ui.Label("Linear Speed (rad/s)",
                     style={"color": 0xFFAAAAAA, "font_size": 12})
            lin = ui.FloatSlider(min=0.5, max=15.0, step=0.5)
            lin.model.set_value(state.get("linear_speed", 5.0))

            def on_linear(m):
                v = m.get_value_as_float()
                state["linear_speed"] = v
                if state["controller"]:
                    state["controller"]._linear = v

            lin.model.add_value_changed_fn(on_linear)

            ui.Spacer(height=4)
            ui.Label("Angular Speed (rad/s)",
                     style={"color": 0xFFAAAAAA, "font_size": 12})
            ang = ui.FloatSlider(min=0.5, max=10.0, step=0.5)
            ang.model.set_value(state.get("angular_speed", 3.0))

            def on_angular(m):
                v = m.get_value_as_float()
                state["angular_speed"] = v
                if state["controller"]:
                    state["controller"]._angular = v

            ang.model.add_value_changed_fn(on_angular)

            ui.Spacer(height=8)

            with ui.HStack(spacing=8):
                ui.Label("Keyboard Control", width=140)
                kb = ui.CheckBox()
                kb.model.set_value(state.get("keyboard_enabled", True))
                kb.model.add_value_changed_fn(
                    lambda m: state.update({"keyboard_enabled": m.get_value_as_bool()})
                )

            with ui.HStack(spacing=8):
                ui.Label("Telemetry", width=140)
                tel = ui.CheckBox()
                tel.model.set_value(state.get("telemetry_enabled", True))
                tel.model.add_value_changed_fn(
                    lambda m: state.update({"telemetry_enabled": m.get_value_as_bool()})
                )
