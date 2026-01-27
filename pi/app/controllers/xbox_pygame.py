from __future__ import annotations
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import pygame
from app.utils import clamp
from app.controllers.types import Controller, ControllerState

os.environ["SDL_VIDEODRIVER"] = "dummy"

@dataclass
class RawXboxState:
    axes: Dict[str, float]
    buttons: Dict[str, int]
    hat: Tuple[int, int]


class XboxController:
    """
    Raw pygame joystick reader. Returns axis/button dictionaries.
    """
    DEFAULT_AXIS_MAP = {
        "LX": 0,
        "LY": 1,
        "LT": 2,
        "RY": 3,
        "RT": 5,
    }

    DEFAULT_BUTTON_MAP = {
        "A": 0,
        "B": 1,
        "X": 2,
        "Y": 3,
        "LB": 4,
        "RB": 5,
        "VIEW": 6,
        "MENU": 7,
    }

    def __init__(
        self,
        index: int = 0,
        deadzone: float = 0.12,
        axis_map: Optional[Dict[str, int]] = None,
        button_map: Optional[Dict[str, int]] = None,
    ):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() <= index:
            raise RuntimeError("No controller found. Is it on and connected?")

        self.js = pygame.joystick.Joystick(index)
        self.js.init()

        self.deadzone = deadzone
        self.axis_map = axis_map or dict(self.DEFAULT_AXIS_MAP)
        self.button_map = button_map or dict(self.DEFAULT_BUTTON_MAP)

    def _dz(self, x: float) -> float:
        return 0.0 if abs(x) < self.deadzone else x

    def read_raw(self) -> RawXboxState:
        pygame.event.pump()

        axes: Dict[str, float] = {}
        for name, idx in self.axis_map.items():
            if idx < self.js.get_numaxes():
                axes[name] = self._dz(float(self.js.get_axis(idx)))

        buttons: Dict[str, int] = {}
        for name, idx in self.button_map.items():
            if idx < self.js.get_numbuttons():
                buttons[name] = int(self.js.get_button(idx))

        hat = (0, 0)
        if self.js.get_numhats() > 0:
            hat = self.js.get_hat(0)

        return RawXboxState(axes=axes, buttons=buttons, hat=hat)


class XboxPygameAdapter(Controller):
    """
    Normalizes RawXboxState -> ControllerState (our app-level contract).
    """
    def __init__(self, ctrl: XboxController):
        self.ctrl = ctrl

    def read_state(self) -> ControllerState:
        st = self.ctrl.read_raw()

        # Pygame LY: up is -1.0, we want forward positive.
        ly = float(st.axes.get("LY", 0.0))
        lx = float(st.axes.get("LX", 0.0))

        forward = clamp(-ly, -1.0, 1.0)
        turn = clamp(lx, -1.0, 1.0)

        def norm_trigger(x: float) -> float:
            # Heuristic: sometimes triggers appear as -1..1. Map to 0..1.
            x = clamp(x, -1.0, 1.0)
            if x < 0.0:
                return (x + 1.0) * 0.5
            return clamp(x, 0.0, 1.0)

        lt = norm_trigger(float(st.axes.get("LT", 0.0)))
        rt = norm_trigger(float(st.axes.get("RT", 0.0)))

        def btn(name: str) -> bool:
            return bool(st.buttons.get(name, 0))

        return ControllerState(
            forward=forward,
            turn=turn,
            lt=lt,
            rt=rt,
            a=btn("A"),
            b=btn("B"),
            x=btn("X"),
            y=btn("Y"),
            lb=btn("LB"),
            rb=btn("RB"),
            menu=btn("MENU"),
            view=btn("VIEW"),
        )