"""ConsoleManager – timestamped rolling log with duplicate suppression."""

from datetime import datetime

CONSOLE_MAX_LINES = 10


class ConsoleManager:

    def __init__(self, history, label_getter):
        """
        Args:
            history: list shared with UIBuilder for console entries.
            label_getter: callable returning the current console ui.Label.
        """
        self._history = history
        self._label_getter = label_getter

    def log(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        entry = f"[{ts}] {message}"

        # suppress duplicate consecutive messages
        if self._history and self._history[-1].endswith(message):
            return

        self._history.append(entry)
        if len(self._history) > CONSOLE_MAX_LINES:
            self._history.pop(0)

        label = self._label_getter()
        if label is not None:
            label.text = "\n".join(self._history)

    def clear(self) -> None:
        self._history.clear()
        label = self._label_getter()
        if label is not None:
            label.text = ""
