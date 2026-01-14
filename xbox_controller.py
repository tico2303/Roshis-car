import os
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple
import pygame

os.environ["SDL_VIDEODRIVER"] = "dummy"

@dataclass
class XboxControllerState:
    axes: Dict[str, float]
    buttons: Dict[str, int]
    hat: Tuple[int, int]

class XboxController:
    """
    Lightweight controller wrapper using pygame.
    Mapping defaults match common Xbox 360-style layouts in pygame docs,
    but you can override indices per-controller.
    """
    DEFAULT_AXIS_MAP = {
        "LX": 0, 
        "LY": 1,
        "LT": 5,  # sometimes 2
        "RX": 2, 
        "RY": 3,
        "RT": 4,  # sometimes 5
    }

    DEFAULT_BUTTON_MAP = {
        "A": 0, 
        "B": 1, 
        "X": 3, 
        "Y": 4,
        "LB": 6, 
        "RB": 7,
        "MENU": 11, 
        "VIEW": 15,
        "L_PRESS": 13, 
        "R_PRESS": 14,
        # Guide/Xbox button is often not exposed on Linux drivers
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
            raise RuntimeError("No controller found at that index.")

        self.js = pygame.joystick.Joystick(index)
        self.js.init()

        self.deadzone = deadzone
        self.axis_map = axis_map or dict(self.DEFAULT_AXIS_MAP)
        self.button_map = button_map or dict(self.DEFAULT_BUTTON_MAP)

        self._prev_axes: Dict[str, float] = {}
        self._prev_buttons: Dict[str, int] = {}

        self.on_axis: Optional[Callable[[str, float], None]] = None
        self.on_button: Optional[Callable[[str, int], None]] = None

    def _dz(self, x: float) -> float:
        return 0.0 if abs(x) < self.deadzone else x

    def read(self) -> XboxControllerState:
        pygame.event.pump()

        axes = {}
        for name, idx in self.axis_map.items():
            if idx < self.js.get_numaxes():
                axes[name] = self._dz(float(self.js.get_axis(idx)))

        buttons = {}
        for name, idx in self.button_map.items():
            if idx < self.js.get_numbuttons():
                buttons[name] = int(self.js.get_button(idx))

        hat = (0, 0)
        if self.js.get_numhats() > 0:
            hat = self.js.get_hat(0)

        return XboxControllerState(axes=axes, buttons=buttons, hat=hat)

    def poll(self, hz: float = 100.0):
        period = 1.0 / hz
        while True:
            st = self.read()

            # fire callbacks on changes
            for k, v in st.axes.items():
                if self._prev_axes.get(k) != v:
                    self._prev_axes[k] = v
                    if self.on_axis:
                        self.on_axis(k, v)

            for k, v in st.buttons.items():
                if self._prev_buttons.get(k) != v:
                    self._prev_buttons[k] = v
                    if self.on_button:
                        self.on_button(k, v)

            time.sleep(period)


# ----------------------------
# Controller abstraction
# ----------------------------
@dataclass(frozen=True)
class ControllerState:
    """
    Normalized controller state.
    Axes: floats in [-1.0, 1.0]
    Buttons: bool
    """
    ly: float  # left stick Y (forward/back) convention: up is -1 on many libs
    lx: float  # left stick X (left/right) convention: up is -1 on many libs
    a: bool
    b: bool
    x: bool
    y: bool
    lb: bool
    rb: bool
    start: bool
    back: bool


class XboxControllerAdapter:
    """
    Adapter around your XboxController class.
    You MUST edit `read_state()` to match your XboxController API.
    """
    def __init__(self) -> None:
        self.ctrl = XboxController()

    def read_state(self) -> ControllerState:
        """
        TODO: Modify this to match your XboxController implementation.
        Below are placeholder attribute names.
        """
        # Common patterns (examples):
        # - self.ctrl.ly
        # - self.ctrl.get_left_stick_y()
        # - self.ctrl.state["ly"]
        #
        # Replace these with your real accessors:
        ly = float(getattr(self.ctrl, "ly", 0.0))

        def btn(name: str) -> bool:
            return bool(getattr(self.ctrl, name, False))

        return ControllerState(
            ly=ly,
            a=btn("a"),
            b=btn("b"),
            x=btn("x"),
            y=btn("y"),
            lb=btn("lb"),
            rb=btn("rb"),
            start=btn("start"),
            back=btn("back"),
        )


"""
import pygame
pygame.init()
pygame.joystick.init()

js = pygame.joystick.Joystick(0)
js.init()

print(js.get_name())
print("axes:", js.get_numaxes())
print("buttons:", js.get_numbuttons())
print("hats:", js.get_numhats())

import time
while True:
    pygame.event.pump()
    print([js.get_axis(i) for i in range(js.get_numaxes())],
          [js.get_button(i) for i in range(js.get_numbuttons())])
    time.sleep(0.2)

"""

if __name__ == "__main__":
    c = XboxController(deadzone=0.12)

    def axis_cb(name, val):
        print(f"AXIS {name}: {val:+.3f}")

    def button_cb(name, val):
        print(f"BUTTON {name}: {'DOWN' if val else 'UP'}")

    c.on_axis = axis_cb
    c.on_button = button_cb
    c.poll(hz=100)

