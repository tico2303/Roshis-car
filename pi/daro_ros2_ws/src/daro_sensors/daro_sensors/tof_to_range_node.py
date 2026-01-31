import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
import math
import json
from builtin_interfaces.msg import Time
from rclpy.time import Time as RosTime


class TofToRangeNode(Node):
    def __init__(self):
        super().__init__('tof_to_range_node')
         # -------- Params --------
        self.declare_parameter("pub_topic", "/tof/range")
        self.declare_parameter("rx_topic", "/esp32/rx_json/tof")

        self.declare_parameter("frame_id", "tof_link")
        self.declare_parameter("tof_min_range_m", 0.035)
        self.declare_parameter("tof_max_range_m", 0.35)
        self.declare_parameter("tof_fov_rad", 0.26)
        self.declare_parameter("use_ros_time", True)

        # subscriptions
        sub_topic = str(self.get_parameter("rx_topic").value)
        que_depth = 10
        self.create_subscription(String, sub_topic,self.on_tof_callback, que_depth)

        #publish
        pub_topic = str(self.get_parameter("pub_topic").value)
        self.publish_range = self.create_publisher(Range, pub_topic, que_depth)

    
    def _parse_tof_json_to_range(self,
        json_str: str,
        frame_id: str,
        min_range_m: float,
        max_range_m: float,
        fov_rad: float,
        use_ros_time: bool,
    ) -> Range:
        """
        Parse a ToF NDJSON message and return a sensor_msgs/Range.

        Expected JSON format:
        {
        "type": "tof",
        "seq": 3063,
        "id": 0,
        "mm": 8190,
        "status": 0,
        "ts_ms": 153722
        }
        """

        msg = Range()

        msg.header.frame_id = frame_id

        if use_ros_time:
            msg.header.stamp = RosTime().to_msg()
        else:
            # convert ESP32 timestamp ms to ros Time
            t = Time()
            t.sec = int(json_data["ts_ms"] / 1000)
            t.nanosec = int((json_data["ts_ms"] % 1000) * 1e6)
            msg.header.stamp = t

        # --- Static sensor metadata ---
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = fov_rad
        msg.min_range = min_range_m
        msg.max_range = max_range_m

        json_data = json.loads(json_str)

        status = json_data.get("status", -1)
        mm = json_data.get("mm", None)

        # --- Range value ---
        if status != 0 or mm is None:
            # Invalid reading is nan
            msg.range = math.nan
        else:
            range_m = mm / 1000.0

            # Clamp to declared limits
            if range_m < min_range_m or range_m > max_range_m:
                msg.range = math.nan
            else:
                msg.range = range_m

        return msg

    def on_tof_callback(self, msg: String):
        frame_id = str(self.get_parameter("frame_id").value)
        min_range_m = float(self.get_parameter("tof_min_range_m").value)
        max_range_m = float(self.get_parameter("tof_max_range_m").value)
        fov_rad = float(self.get_parameter("tof_fov_rad").value)
        use_ros_time = bool(self.get_parameter("use_ros_time").value)

        try:
            range_msg = self._parse_tof_json_to_range(
                json_str=msg.data,
                frame_id=frame_id,
                min_range_m=min_range_m,
                max_range_m=max_range_m,
                fov_rad=fov_rad,
                use_ros_time=use_ros_time,
            )
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON on /esp32/rx_json/tof: {e}")
            return
        except Exception as e:
            self.get_logger().warn(f"Failed to parse ToF: {e}")
            return

        self.publish_range.publish(range_msg)


def main():
    rclpy.init()
    node = None
    try:
        node = TofToRangeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()