#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


@dataclass(frozen=True)
class ButtonBinding:
    name: str
    index: int
    topic: str




class JoyButtonsNode(Node):
    """
    Subscribes to /joy and publishes edge-triggered button events.

    - Mapping is loaded from a separate YAML file (dict-friendly).
    - Publishes std_msgs/Empty on /daro/events/<name> when button is pressed.
    """

    def __init__(self) -> None:
        super().__init__("joy_buttons")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("events_ns", "/daro/events")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("publish_on_press", True)

        joy_topic = str(self.get_parameter("joy_topic").value)
        events_ns = str(self.get_parameter("events_ns").value).rstrip("/")
        mapping_file = str(self.get_parameter("mapping_file").value)
        publish_on_press = bool(self.get_parameter("publish_on_press").value)

        if not mapping_file:
            raise RuntimeError("mapping_file parameter is empty. Set it in joy_buttons_params.yaml.")

        button_map = self._load_button_map(mapping_file)

        # publishers per binding
        self._bindings: Dict[str, ButtonBinding] = {}
        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}

        for name, idx in button_map.items():
            topic = f"{events_ns}/{name}"
            self._bindings[name] = ButtonBinding(name=name, index=idx, topic=topic)
            self._pubs[name] = self.create_publisher(Empty, topic, 10)

        self._last_buttons: Optional[List[int]] = None
        self._publish_on_press = publish_on_press

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._sub = self.create_subscription(Joy, joy_topic, self._on_joy, qos)

        self.get_logger().info(f"Loaded mapping_file: {mapping_file}")
        self.get_logger().info(f"Subscribing to: {joy_topic}")
        self.get_logger().info(f"Publishing under: {events_ns}/<name>")
        self.get_logger().info(
            "Bindings: " + ", ".join([f"{b.name}=buttons[{b.index}] -> {b.topic}" for b in self._bindings.values()])
        )

    def _on_joy(self, msg: Joy) -> None:
        buttons = [int(b) for b in msg.buttons]

        if self._last_buttons is None:
            self._last_buttons = buttons
            return

        for name, binding in self._bindings.items():
            idx = binding.index
            if idx < 0 or idx >= len(buttons) or idx >= len(self._last_buttons):
                continue

            prev = self._last_buttons[idx]
            cur = buttons[idx]

            fired = (prev == 0 and cur == 1) if self._publish_on_press else (prev != cur)
            if fired:
                self._pubs[name].publish(Empty())
                self.get_logger().info(f"event: {name}")

        self._last_buttons = buttons

    def _load_button_map(self,path: str) -> Dict[str, int]:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        buttons = data.get("buttons")
        if not isinstance(buttons, dict) or not buttons:
            raise RuntimeError(f"{path} must contain a top-level 'buttons:' mapping")

        out: Dict[str, int] = {}
        for name, idx in buttons.items():
            if not isinstance(name, str) or not name.strip():
                raise RuntimeError(f"Invalid button name in {path}: {name!r}")
            try:
                out[name.strip()] = int(idx)
            except Exception as e:
                raise RuntimeError(f"Button '{name}' index must be int in {path}, got {idx!r}") from e

        return out

def main() -> None:
    rclpy.init()
    node = None
    try:
        node = JoyButtonsNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()