from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ControllerState:
    """
    Normalized controller state.

    Conventions:
      - forward: +1.0 forward, -1.0 backward
      - turn: +1.0 right, -1.0 left
      - lt/rt: 0.0..1.0
    """
    forward: float
    turn: float
    lt: float
    rt: float

    a: bool
    b: bool
    x: bool
    y: bool
    lb: bool
    rb: bool
    menu: bool
    view: bool


class Controller(Protocol):
    def read_state(self) -> ControllerState: ...