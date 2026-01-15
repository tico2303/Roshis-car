import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

js = pygame.joystick.Joystick(0)
js.init()

print(f"Joystick: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")

try:
    while True:
        pygame.event.pump()

        axes = [js.get_axis(i) for i in range(js.get_numaxes())]
        buttons = [js.get_button(i) for i in range(js.get_numbuttons())]

        print("Axes:", ["%+.2f" % a for a in axes], "Buttons:", buttons)
        time.sleep(0.1)

except KeyboardInterrupt:
    pass