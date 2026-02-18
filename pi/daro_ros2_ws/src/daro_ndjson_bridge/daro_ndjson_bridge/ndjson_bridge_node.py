#!/usr/bin/env python3
from __future__ import annotations

import json
import queue
import re
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


_TYPE_SAFE_RE = re.compile(r"[^a-z0-9_]+")


def _crc16(data: bytes) -> int:
    """CRC-16/XMODEM (poly 0x1021, init 0x0000) — matches ESP32 sendJson."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _cobs_decode(data: bytes) -> bytes:
    """Decode a COBS-encoded frame (without the trailing 0x00 delimiter)."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0:
            return b""  # unexpected null — corrupt
        for _ in range(code - 1):
            if i >= len(data):
                return b""
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def _cobs_encode(data: bytes) -> bytes:
    """COBS-encode data, returns encoded bytes including trailing 0x00."""
    out = bytearray()
    idx = 0
    code_idx = len(out)
    out.append(0)  # placeholder for first code byte
    code = 1
    for b in data:
        if b != 0:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
        else:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
    out[code_idx] = code
    out.append(0x00)  # frame delimiter
    return bytes(out)


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

        # RX uses a manual byte buffer instead of readline() for
        # reliable framing even when pyserial returns partial reads.
        self._rx_buf = bytearray()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        # TX queue — all serial writes go through the RX thread to avoid
        # concurrent read/write on the CP2102, which causes byte loss.
        self._tx_queue: queue.Queue[bytes] = queue.Queue(maxsize=200)
        self._tx_count_10s = 0

        # Drop stats — logged periodically so you can monitor link quality
        self._drop_cobs_fail = 0
        self._drop_no_tab = 0
        self._drop_bad_crc_hex = 0
        self._drop_crc_mismatch = 0
        self._drop_newlines_seen = 0
        self._rx_good = 0
        self._crc_log_timer = self.create_timer(10.0, self._log_crc_stats)

        self._connect()
        self._rx_thread.start()

        self.get_logger().info(
            f"NDJSON bridge started (COBS framing). "
            f"RX root={self.rx_root}, TX topic={self.tx_topic}"
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
                exclusive=True,  # TIOCEXCL — prevents other processes opening the port
            )
            # Flush any stale partial data sitting in the OS buffer
            ser.reset_input_buffer()
            with self._ser_lock:
                self._ser = ser
            # Clear the RX line buffer on fresh connection so we don't
            # try to parse a leftover partial line from a previous session
            self._rx_buf.clear()
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
        """Enqueue a TX frame; actual write happens in _rx_loop to avoid
        concurrent read/write on the USB-serial chip."""
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

        compact = json.dumps(obj, separators=(",", ":"))
        crc = _crc16(compact.encode("utf-8"))
        payload = f"{compact}\t{crc:04x}".encode("utf-8")
        frame = _cobs_encode(payload)

        try:
            self._tx_queue.put_nowait(frame)
        except queue.Full:
            pass  # drop oldest commands silently under load

    # ---------------- RX: Serial -> ROS ----------------
    def _get_or_create_type_pub(self, safe_type: str) -> Optional[rclpy.publisher.Publisher]:
        if safe_type in self._pub_by_type:
            return self._pub_by_type[safe_type]

        max_types = int(self.get_parameter("max_dynamic_types").value)
        if len(self._pub_by_type) >= max_types:
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
        """
        Read bytes from serial into a buffer, extract complete newline-
        delimited lines, parse as JSON, and publish to ROS topics.

        This approach is resilient to:
        - Partial reads (pyserial returning fewer bytes than a full line)
        - The "readiness but no data" transient USB-serial condition
        - Buffer boundaries falling mid-message
        """
        reconnect_s = int(self.get_parameter("reconnect_sleep_ms").value) / 1000.0
        max_buf = 4096  # safety cap — discard buffer if it grows too large

        while not self._stop.is_set():
            self._ensure_connected()
            with self._ser_lock:
                ser = self._ser

            if ser is None:
                time.sleep(reconnect_s)
                continue

            try:
                # --- Drain TX queue (write before read so ESP32 gets
                # commands promptly, and all I/O is on this one thread) ---
                while not self._tx_queue.empty():
                    try:
                        frame = self._tx_queue.get_nowait()
                        ser.write(frame)
                        self._tx_count_10s += 1
                    except queue.Empty:
                        break
                    except Exception as e:
                        self.get_logger().warn(f"Serial write failed: {e}")
                        self._close_serial()
                        break

                # --- Read whatever bytes are available (up to 512). ---
                chunk = ser.read(ser.in_waiting or 1)

                if not chunk:
                    continue

                self._rx_buf.extend(chunk)

                # Safety: if buffer grows huge, something is wrong — discard
                if len(self._rx_buf) > max_buf:
                    self.get_logger().warn(
                        f"RX buffer overflow ({len(self._rx_buf)} bytes), discarding"
                    )
                    self._rx_buf.clear()
                    continue

                # Track newlines — if we see them, ESP32 is sending old firmware
                nl_count = chunk.count(b"\n")
                if nl_count:
                    self._drop_newlines_seen += nl_count

                # Extract and process all complete COBS frames (delimited by 0x00)
                while b"\x00" in self._rx_buf:
                    idx = self._rx_buf.index(b"\x00")
                    raw_frame = bytes(self._rx_buf[:idx])
                    del self._rx_buf[: idx + 1]

                    if not raw_frame:
                        continue

                    line = _cobs_decode(raw_frame)
                    if not line:
                        self._drop_cobs_fail += 1
                        continue

                    self._process_rx_line(line)

            except serial.SerialException as e:
                err_str = str(e)
                if "returned no data" in err_str or "readiness" in err_str:
                    # Transient USB-serial glitch — NOT a real disconnect.
                    # Just retry on next loop iteration.
                    time.sleep(0.01)
                    continue

                # Real serial error — reconnect
                self.get_logger().warn(f"Serial error: {e} — reconnecting")
                self._close_serial()
                self._rx_buf.clear()
                time.sleep(reconnect_s)
            except OSError as e:
                self.get_logger().warn(f"Serial OS error: {e} — reconnecting")
                self._close_serial()
                self._rx_buf.clear()
                time.sleep(reconnect_s)
            except Exception as e:
                self.get_logger().warn(f"RX unexpected error: {e}")
                time.sleep(0.05)

    def _process_rx_line(self, line: bytes) -> None:
        """Parse one complete NDJSON line and publish to ROS topics.

        Expected wire format: <json>\\t<2-hex-CRC8>
        If a tab is present the CRC is verified; corrupted lines are
        silently dropped.  Lines without a tab are treated as legacy
        (no checksum) and parsed directly.
        """
        # --- CRC verification ---
        tab_idx = line.rfind(b"\t")
        if tab_idx < 0:
            self._drop_no_tab += 1
            return

        json_part = line[:tab_idx]
        crc_hex = line[tab_idx + 1:]
        try:
            expected = int(crc_hex, 16)
        except ValueError:
            self._drop_bad_crc_hex += 1
            return
        actual = _crc16(json_part)
        if actual != expected:
            self._drop_crc_mismatch += 1
            return
        line = json_part

        try:
            obj = json.loads(line.decode("utf-8", errors="replace"))
            if not isinstance(obj, dict):
                raise ValueError("rx must be JSON object")
        except Exception as e:
            # This should be very rare now — CRC passed but JSON invalid
            if bool(self.get_parameter("log_rx_bad_json").value):
                self.get_logger().warn(
                    f"RX CRC OK but invalid JSON: {e} | {line[:200]!r}",
                    throttle_duration_sec=2.0,
                )
            return

        self._rx_good += 1

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

    def _log_crc_stats(self) -> None:
        total_drops = (
            self._drop_cobs_fail
            + self._drop_no_tab
            + self._drop_bad_crc_hex
            + self._drop_crc_mismatch
        )
        if total_drops > 0 or self._rx_good > 0:
            parts = []
            if self._rx_good:
                parts.append(f"ok={self._rx_good}")
            if self._drop_cobs_fail:
                parts.append(f"cobs_fail={self._drop_cobs_fail}")
            if self._drop_no_tab:
                parts.append(f"no_tab={self._drop_no_tab}")
            if self._drop_bad_crc_hex:
                parts.append(f"bad_crc_hex={self._drop_bad_crc_hex}")
            if self._drop_crc_mismatch:
                parts.append(f"crc_mismatch={self._drop_crc_mismatch}")
            if self._drop_newlines_seen:
                parts.append(f"NEWLINES={self._drop_newlines_seen}(!)")
            if self._tx_count_10s:
                parts.append(f"tx={self._tx_count_10s}")

            level = "info" if total_drops == 0 else "warn"
            msg = f"Serial 10s: {' | '.join(parts)}"

            if self._drop_newlines_seen > 0 and self._rx_good == 0:
                msg += " >>> ESP32 sending newline-framed data — reflash firmware!"
            elif self._drop_no_tab > total_drops * 0.5:
                msg += " >>> Most frames have no tab — possible framing mismatch"

            getattr(self.get_logger(), level)(msg)

        self._drop_cobs_fail = 0
        self._drop_no_tab = 0
        self._drop_bad_crc_hex = 0
        self._drop_crc_mismatch = 0
        self._drop_newlines_seen = 0
        self._rx_good = 0
        self._tx_count_10s = 0

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
