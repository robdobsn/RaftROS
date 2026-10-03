# Count samples per minute on the device's range and /chatter topics for a fixed
# time, one JSON line per minute, then exit by itself.
# Usage: counter.py <seconds> <out.jsonl>
import json, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String
dur, path = float(sys.argv[1]), sys.argv[2]
class Counter(Node):
    def __init__(self):
        super().__init__("raftros_counter")
        self.range = 0; self.chatter = 0
        self.create_subscription(Range, "/raft/range_1_29", self.on_range, qos_profile_sensor_data)
        self.create_subscription(String, "/chatter", self.on_chatter, 10)
        self.create_timer(60.0, self.report)
        self.out = open(path, "a", buffering=1)
    def on_range(self, m): self.range += 1
    def on_chatter(self, m): self.chatter += 1
    def report(self):
        self.out.write(json.dumps({"t": int(time.time()), "range": self.range, "chatter": self.chatter}) + "\n")
        self.range = 0; self.chatter = 0
rclpy.init(); node = Counter(); end = time.time() + dur
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=1.0)
node.destroy_node(); rclpy.shutdown()
