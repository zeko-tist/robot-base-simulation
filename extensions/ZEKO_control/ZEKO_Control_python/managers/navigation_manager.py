"""NavigationManager – placeholder for future autonomous navigation."""

DESTINATIONS = ["Waypoint A", "Waypoint B", "Waypoint C", "Home"]


class NavigationManager:

    def __init__(self, console_manager):
        self._console = console_manager
        self._destination = DESTINATIONS[0]
        self._active = False

    def set_destination(self, dest: str) -> None:
        self._destination = dest

    def go(self) -> None:
        if not self._destination:
            self._console.log("Navigation: no destination set.")
            return
        self._active = True
        self._console.log(f"Navigation: going to '{self._destination}'…")
        # TODO: connect to Nav2 / waypoint planner here

    def cancel(self) -> None:
        self._active = False
        self._console.log("Navigation: cancelled.")
