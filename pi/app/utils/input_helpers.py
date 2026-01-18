from typing import Callable, Dict, Optional


class EdgeDetector:
    """
    Tracks button rising edges so actions fire once per press.
    """
    def __init__(self) -> None:
        self._prev: Dict[str, bool] = {}

    def rising(self, key: str, current: bool) -> bool:
        prev = self._prev.get(key, False)
        self._prev[key] = current
        return (not prev) and current


class ActionBindings:
    """
    Bind buttons to actions (extensible).
    Actions are callables that take no args.
    """
    def __init__(self) -> None:
        self._bindings: Dict[str, Callable[[], None]] = {}

    def bind(self, name: str, action: Callable[[], None]) -> None:
        self._bindings[name] = action

    def run(self, name: str) -> None:
        action = self._bindings.get(name)
        if action:
            action()

"""
Facace for Edge Detection and Action Bindings
"""
class InputFacade:
    """
    Facade combining edge detection and action bindings.

    Usage:
      f = InputFacade()
      f.bind('A', lambda: ...)
      f.process('a', current_bool)  # will run bound 'A' on rising edge
    """
    def __init__(self) -> None:
        self.detector = EdgeDetector()
        self.bindings = ActionBindings()

    def bind(self, name: str, action: Callable[[], None]) -> None:
        self.bindings.bind(name, action)

    def process(self, key: str, current: bool, binding_name: Optional[str] = None) -> None:
        if self.detector.rising(key, current):
            name = binding_name if binding_name is not None else key.upper()
            self.bindings.run(name)
