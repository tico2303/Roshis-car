import time

# Import your existing classes
from app.controllers.xbox_controller import XboxController
from app.drive.xbox_drive_bridge import build_drive_bridge
from app.protocol.print_protocol import PrintOnlyProtocol

def main():
    print("Starting Xbox → Drive JSON demo")
    print("Move left stick to see output. Ctrl+C to exit.\n")

    # 1) Create the controller (pygame)
    ctrl = XboxController(index=0)

    # 2) Use print-only protocol instead of serial
    proto = PrintOnlyProtocol()

    # 3) Build the bridge
    bridge = build_drive_bridge(ctrl, proto)

    # 4) Run
    bridge.run_forever()


if __name__ == "__main__":
    main()