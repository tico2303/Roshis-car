#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


_TYPE_SAFE_RE = re.compile(r"[^a-z0-9_]+")


def sanitize_type(t: str) -> str:
    """
    ROS topic segments should be conservative.
    Convert to lowercase, replace unsafe chars with underscore, trim.
    """
    t = t.strip().lower()
    t = _TYPE_SAFE_RE.sub("_", t)
    t = t.strip("_")
    return t or "unknown"


class NdjsonBridgeNode(Node):
    """
    Transport + routing NDJSON <-> ROS.

    RX:
      Serial NDJSON object -> publish JSON text on:
        /esp32/rx_json/<type>
      and also on:
        /esp32/rx_json/all

    TX:
      Subscribe to /esp32/tx_json (String containing JSON object text) -> write NDJSON line.

    Bridge does NOT interpret message contents beyond:
      - must be JSON object
      - must have "type" field (string) for typed topics
    """

    def __init__(self) -> None:
        super().__init__("ndjson_bridge")

        # -------- Params --------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_sleep_ms", 200)

        self.declare_parameter("log_rx_bad_json", True)
        self.declare_parameter("log_tx_bad_json", True)

        # Safety: prevent spamming infinite new topic types
        self.declare_parameter("max_dynamic_types", 64)

        # Topic roots
        self.declare_parameter("rx_root", "/esp32/rx_json")
        self.declare_parameter("tx_topic", "/esp32/tx_json")

        # Serial read timeout — must be long enough for the longest
        # NDJSON line to arrive fully at the configured baud rate.
        self.declare_parameter("serial_timeout_s", 0.5)

        self.rx_root = str(self.get_parameter("rx_root").value).rstrip("/")
        self.tx_topic = str(self.get_parameter("tx_topic").value)

        # -------- Publishers (dynamic by type) --------
        self._pub_by_type: Dict[str, rclpy.publisher.Publisher] = {}
        self.pub_all = self.create_publisher(String, f"{self.rx_root}/all", 50)

        # -------- TX subscription --------
        self.sub_tx = self.create_subscription(String, self.tx_topic, self._on_tx_json, 50)

        # -------- Serial state --------
        self._ser_lock = threading.Lock()
        self._ser: Optional[serial.Serial] = None
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        self._connect()
        self._rx_thread.start()

        self.get_logger().info(
            f"NDJSON bridge started. RX root={self.rx_root}, TX topic={self.tx_topic}"
        )

    # ---------------- Serial management
    def _connect(self) -> None:
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        timeout = float(self.get_parameter("serial_timeout_s").value)
        try:
            ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=timeout,
                write_timeout=0.1,
            )
            # Flush any stale partial data sitting in the OS buffer
            ser.reset_input_buffer()
            with self._ser_lock:
                self._ser = ser
            self.get_logger().info(f"Connected serial: {port} @ {baud}")
        except Exception as e:
            with self._ser_lock:
                self._ser = None
            self.get_logger().warn(f"Serial connect failed ({port}): {e}")

    def _ensure_connected(self) -> None:
        with self._ser_lock:
            ok = self._ser is not None and self._ser.is_open
        if ok:
            return
        self._connect()

    def _close_serial(self) -> None:
        with self._ser_lock:
            ser = self._ser
            self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    # ---------------- TX: ROS -> Serial ----------------
    def _on_tx_json(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        try:
            obj = json.loads(text)
            if not isinstance(obj, dict):
                raise ValueError("tx_json must be a JSON object")
        except Exception as e:
            if bool(self.get_parameter("log_tx_bad_json").value):
                self.get_logger().warn(f"TX invalid JSON: {e} | {text[:160]!r}")
            return

        self._ensure_connected()
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            return

        line = json.dumps(obj, separators=(",", ":")) + "\n"
        try:
            ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")
            self._close_serial()

    # ---------------- RX: Serial -> ROS ----------------
    def _get_or_create_type_pub(self, safe_type: str) -> Optional[rclpy.publisher.Publisher]:
        if safe_type in self._pub_by_type:
            return self._pub_by_type[safe_type]

        max_types = int(self.get_parameter("max_dynamic_types").value)
        if len(self._pub_by_type) >= max_types:
            # Drop new types once limit is reached (prevents runaway topic creation)
            self.get_logger().warn(
                f"Type publisher limit reached ({max_types}). Dropping new type: {safe_type}"
            )
            return None

        topic = f"{self.rx_root}/{safe_type}"
        pub = self.create_publisher(String, topic, 50)
        self._pub_by_type[safe_type] = pub
        self.get_logger().info(f"Created RX topic: {topic}")
        return pub

    def _rx_loop(self) -> None:
        reconnect_s = int(self.get_parameter("reconnect_sleep_ms").value) / 1000.0
        consecutive_errors = 0
        max_errors_before_reconnect = 5

        while not self._stop.is_set():
            self._ensure_connected()
            with self._ser_lock:
                ser = self._ser

            if ser is None:
                time.sleep(reconnect_s)
                continue

            try:
                # readline() blocks until \n or timeout.
                # With a 0.5s timeout, even long NDJSON lines (enc+imu)
                # have plenty of time to arrive at 115200 baud.
                raw = ser.readline()

                if not raw:
                    # Timeout with no data — this is normal when ESP32
                    # isn't sending. Do NOT close the port.
                    continue

                raw = raw.strip()
                if not raw:
                    continue

                # Reset error counter on any successful read
                consecutive_errors = 0

                # Parse JSON object
                try:
                    obj = json.loads(raw.decode("utf-8", errors="replace"))
                    if not isinstance(obj, dict):
                        raise ValueError("rx must be JSON object")
                except Exception as e:
                    if bool(self.get_parameter("log_rx_bad_json").value):
                        self.get_logger().warn(
                            f"RX invalid JSON: {e} | {raw[:200]!r}",
                            throttle_duration_sec=2.0,
                        )
                    continue

                # Canonical JSON string output
                payload = json.dumps(obj, separators=(",", ":"))

                # Publish to /all
                msg_all = String()
                msg_all.data = payload
                self.pub_all.publish(msg_all)

                # Publish to /<type>
                t = obj.get("type")
                if isinstance(t, str) and t.strip():
                    safe_t = sanitize_type(t)
                else:
                    safe_t = "unknown"

                pub = self._get_or_create_type_pub(safe_t)
                if pub is not None:
                    m = String()
                    m.data = payload
                    pub.publish(m)

            except serial.SerialException as e:
                consecutive_errors += 1
                if consecutive_errors >= max_errors_before_reconnect:
                    self.get_logger().warn(
                        f"Serial error ({consecutive_errors} consecutive): {e} — reconnecting"
                    )
                    self._close_serial()
                    time.sleep(reconnect_s)
                    consecutive_errors = 0
            except OSError as e:
                self.get_logger().warn(f"Serial OS error: {e} — reconnecting")
                self._close_serial()
                time.sleep(reconnect_s)
            except Exception as e:
                self.get_logger().warn(f"RX unexpected error: {e}")
                time.sleep(0.05)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = NdjsonBridgeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
