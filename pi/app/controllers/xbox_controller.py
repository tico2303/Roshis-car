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
    Mapping defaults match common Xbox controller in pygame
    """
    DEFAULT_AXIS_MAP = {
        "LX": 0, 
        "LY": 1,
        "LT": 2, 
        "RX": 2, 
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
        "MENU": 7, 
        "VIEW": 6,
        "L_PRESS": 9, 
        "R_PRESS": 10,
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
            print("Is Controller On and Paired?")
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


class XboxControllerAdapter:
    """
    Adapter around your XboxController class.
    You MUST edit `read_state()` to match your XboxController API.
    """
    def __init__(self) -> None:
        self.ctrl = XboxController()

    def read_state(self) -> ControllerState:
        """
        Read the latest state from the wrapped `XboxController` and
        normalize it into `ControllerState`.
        """
        st = self.ctrl.read()

        # Axes in the underlying wrapper use uppercase names like 'LY', 'LX'
        ly = float(st.axes.get("LY", 0.0))
        lx = float(st.axes.get("LX", 0.0))
        lt = float(st.axes.get("LT", 0.0))
        rt = float(st.axes.get("RT", 0.0))

        # Buttons mapping uses uppercase names as defined in DEFAULT_BUTTON_MAP
        def b(key: str) -> bool:
            return bool(st.buttons.get(key, 0))

        return ControllerState(
            ly=ly,
            lx=lx,
            lt=lt,
            rt=rt,
            a=b("A"),
            b=b("B"),
            x=b("X"),
            y=b("Y"),
            lb=b("LB"),
            rb=b("RB"),
            menu=b("MENU"),
            view=b("VIEW"),
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
    try:
        c = XboxController(deadzone=0.12)

        def axis_cb(name, val):
            print(f"AXIS {name}: {val:+.3f}")

        def button_cb(name, val):
            print(f"BUTTON {name}: {'DOWN' if val else 'UP'}")

        c.on_axis = axis_cb
        c.on_button = button_cb
        c.poll(hz=100)
    except RuntimeError as e:
        print("No joystick/controller detected:", e)
        print("Connect a joystick and re-run, or instantiate `XboxController` with a valid index.")

