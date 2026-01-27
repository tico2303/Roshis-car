#!/usr/bin/env python3
"""
Interactive Xbox controller pairing wizard for Raspberry Pi (BlueZ bluetoothctl).

What it does:
- Ensures bluetooth service is running
- Starts bluetoothctl session (non-blocking; won't hang)
- Turns power on, sets agent, enables pairing/discoverable
- Scans, shows discovered devices
- Prompts you to select the controller
- Optionally removes old record (helps when re-pairing)
- Pairs, trusts, connects using state-based waiting (slower + more reliable)
"""

from __future__ import annotations

import os
import re
import select
import shlex
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

BTCTL = "bluetoothctl"

MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
DEVICE_LINE_RE = re.compile(r"^Device\s+((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})\s+(.+)$")

# ----------------------------
# Timing knobs (tune these)
# ----------------------------
SCAN_SECONDS = 12

PAIR_WAIT_S = 8.0          # how long to wait for Paired: yes
CONNECT_WAIT_S = 8.0       # how long to wait for Connected: yes
RETRY_DELAY_S = 1.5        # pause between connect retries
POST_PAIR_SETTLE_S = 1.0   # let BlueZ settle after pair

INFO_POLL_INTERVAL_S = 0.4
CONNECT_ATTEMPTS = 5


# ----------------------------
# Small helpers
# ----------------------------
@dataclass(frozen=True)
class Device:
    mac: str
    name: str


def run_cmd(cmd: List[str], *, sudo: bool = False) -> Tuple[int, str]:
    """
    Run a command and return (exit_code, combined_output).
    Uses list args to avoid shell quirks.
    """
    if sudo and os.geteuid() != 0:
        cmd = ["sudo"] + cmd
    p = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, p.stdout


def ensure_bluetooth_service() -> None:
    code, out = run_cmd(["systemctl", "is-active", "bluetooth"])
    if out.strip() == "active":
        return

    print("Bluetooth service is not active. Starting it (may prompt for sudo)...")
    code, out = run_cmd(["systemctl", "enable", "--now", "bluetooth"], sudo=True)
    print(out)

    code, out = run_cmd(["systemctl", "is-active", "bluetooth"])
    if out.strip() != "active":
        raise RuntimeError(
            "Bluetooth service still not active.\n"
            "Check: sudo systemctl status bluetooth"
        )


def prompt(msg: str) -> None:
    print()
    print(msg)
    input("Press Enter to continue...")


def parse_devices(text: str) -> List[Device]:
    devices: List[Device] = []
    for line in text.splitlines():
        m = DEVICE_LINE_RE.match(line.strip())
        if m:
            devices.append(Device(mac=m.group(1), name=m.group(2).strip()))
    return devices


def dedupe_devices(devs: List[Device]) -> List[Device]:
    seen: Dict[str, Device] = {}
    for d in devs:
        seen[d.mac.lower()] = d

    out: List[Device] = []
    added = set()
    for d in devs:
        k = d.mac.lower()
        if k not in added:
            out.append(seen[k])
            added.add(k)
    return out


def score_device_name(name: str) -> int:
    """
    Higher score = more likely to be the Xbox controller.
    """
    n = name.lower()
    score = 0
    for kw, pts in (
        ("xbox", 100),
        ("wireless controller", 80),
        ("controller", 30),
        ("microsoft", 20),
    ):
        if kw in n:
            score += pts
    return score


def pick_device(devices: List[Device]) -> Optional[Device]:
    if not devices:
        print("\nNo devices found.")
        return None

    print("\nDiscovered devices:")
    for i, d in enumerate(devices, start=1):
        print(f"  [{i}] {d.mac}  {d.name}")

    while True:
        choice = input("\nChoose number, paste MAC, 'r' rescan, 'q' quit: ").strip()
        if choice.lower() == "q":
            return None
        if choice.lower() == "r":
            return Device(mac="__RESCAN__", name="")
        if MAC_RE.match(choice):
            for d in devices:
                if d.mac.lower() == choice.lower():
                    return d
            return Device(mac=choice, name="(manual)")
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(devices):
                return devices[idx - 1]
        print("Invalid selection.")


# ----------------------------
# bluetoothctl driver (non-blocking)
# ----------------------------
class BluetoothCtl:
    """
    Persistent bluetoothctl session.
    Reads stdout non-blocking using select, so it won't hang if bluetoothctl is quiet.
    """
    def __init__(self) -> None:
        self.p = subprocess.Popen(
            [BTCTL],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        if not self.p.stdin or not self.p.stdout:
            raise RuntimeError("Failed to start bluetoothctl")

        os.set_blocking(self.p.stdout.fileno(), False)
        self._drain(0.3)

    def close(self) -> None:
        try:
            self.request("quit", wait_s=0.2)
        except Exception:
            pass
        try:
            self.p.terminate()
        except Exception:
            pass

    def send(self, cmd: str) -> None:
        assert self.p.stdin is not None
        self.p.stdin.write(cmd.strip() + "\n")
        self.p.stdin.flush()

    def request(self, cmd: str, *, wait_s: float = 0.6) -> str:
        # bluetoothctl treats blank line as "just print prompt"; harmless for draining
        self.send(cmd)
        return self._drain(wait_s)

    def _drain(self, wait_s: float) -> str:
        assert self.p.stdout is not None
        end = time.time() + wait_s
        parts: List[str] = []

        while time.time() < end:
            r, _, _ = select.select([self.p.stdout], [], [], 0.05)
            if not r:
                continue
            try:
                chunk = self.p.stdout.read()
            except BlockingIOError:
                chunk = None
            if chunk:
                parts.append(chunk)

        return "".join(parts)


def collect_devices(bt: BluetoothCtl) -> List[Device]:
    """
    Pull device list from bluetoothctl.
    `devices` lists known devices (including previously seen), which is fine.
    """
    out1 = bt.request("devices", wait_s=0.8)
    out2 = bt.request("devices Paired", wait_s=0.8)
    devs = parse_devices(out1) + parse_devices(out2)
    devs = dedupe_devices(devs)
    devs.sort(key=lambda d: score_device_name(d.name), reverse=True)
    return devs


def wait_for_info_state(bt: BluetoothCtl, mac: str, *, want: str, timeout_s: float) -> bool:
    """
    Poll 'info <mac>' until we see the desired state.
    want = 'paired' or 'connected'
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = bt.request(f"info {mac}", wait_s=0.6)
        low = out.lower()

        if want == "paired":
            if "paired: yes" in low:
                return True
        elif want == "connected":
            if "connected: yes" in low:
                return True

        time.sleep(INFO_POLL_INTERVAL_S)
    return False


def try_pair_trust_connect(bt: BluetoothCtl, mac: str) -> bool:
    print("\nPairing... (this can take a few seconds)")
    out = bt.request(f"pair {mac}", wait_s=2.5)
    print(out)

    # Let bluetoothctl/BlueZ finish background work
    time.sleep(POST_PAIR_SETTLE_S)

    print("Waiting for paired state...")
    if wait_for_info_state(bt, mac, want="paired", timeout_s=PAIR_WAIT_S):
        print("INFO: Paired: yes")
    else:
        print("WARN: did not observe Paired: yes within timeout (continuing anyway)")

    print("Trusting...")
    print(bt.request(f"trust {mac}", wait_s=1.2))

    for attempt in range(1, CONNECT_ATTEMPTS + 1):
        print(f"Connecting... (attempt {attempt}/{CONNECT_ATTEMPTS})")
        out = bt.request(f"connect {mac}", wait_s=2.5)
        print(out)

        print("Waiting for connected state...")
        if wait_for_info_state(bt, mac, want="connected", timeout_s=CONNECT_WAIT_S):
            print("INFO: Connected: yes")
            return True

        print(f"Not connected yet. Sleeping {RETRY_DELAY_S:.1f}s before retry...")
        time.sleep(RETRY_DELAY_S)

    return False


# ----------------------------
# Main
# ----------------------------
def main() -> int:
    ensure_bluetooth_service()

    print("Xbox Controller Bluetooth Pairing Wizard (Pi)")
    print("Tips:")
    print("- Put the controller in pairing mode: hold the Pair button until the Xbox logo blinks rapidly.")
    print("- Keep it close to the Pi.")
    print("- If you've paired before, removing the old record often fixes reconnect issues.")

    bt = BluetoothCtl()
    try:
        # Setup BlueZ state
        print(bt.request("power on"))
        print(bt.request("agent on"))
        print(bt.request("default-agent"))
        print(bt.request("discoverable on"))
        print(bt.request("pairable on"))

        prompt("Now put the Xbox controller in pairing mode (logo blinking fast).")

        while True:
            print(f"\nScanning for ~{SCAN_SECONDS} seconds...")
            print(bt.request("scan on", wait_s=0.4))
            time.sleep(SCAN_SECONDS)
            print(bt.request("scan off", wait_s=0.4))

            devices = collect_devices(bt)
            picked = pick_device(devices)

            if picked is None:
                print("Exiting.")
                return 0
            if picked.mac == "__RESCAN__":
                continue

            mac = picked.mac
            print(f"\nUsing: {mac}  {picked.name}")

            ans = input("Remove old record for this MAC before pairing? [Y/n]: ").strip().lower()
            if ans in ("", "y", "yes"):
                print("\nRemoving any old record (safe if not present)...")
                print(bt.request(f"remove {mac}", wait_s=0.9))
                # Give BlueZ a breath after remove
                time.sleep(0.5)

            ok = try_pair_trust_connect(bt, mac)
            if ok:
                print("\n✅ Paired and connected.")
                print("Verify with:")
                print(f"  bluetoothctl info {shlex.quote(mac)}")
                print("\nTest joystick input (optional):")
                print("  sudo apt install -y joystick")
                print("  jstest /dev/input/js0")
                return 0

            print("\n⚠️ Pair/connect did not succeed.")
            prompt("We will rescan. Ensure the controller is still blinking fast (pairing mode).")

    finally:
        bt.close()


if __name__ == "__main__":
    raise SystemExit(main())