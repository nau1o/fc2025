import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import std_msgs

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class vision_test(Node):
    def __init__(self):
        super().__init__('vision_test')
        self.target_pub = self.create_publisher(std_msgs.msg.UInt32, "/vision/target_num",qos_profile,)
        self.create_timer(0.1, self.timer_cb)
        self.targer_sub = self.create_subscription(std_msgs.msg.UInt32, "/vision/target_num", self.target_num_cb, qos_profile)

    def timer_cb(self):
        msg = std_msgs.msg.UInt32()
        msg.data = 100
        self.target_pub.publish(msg)
        print("publish: ", msg.data)
    def target_num_cb(self, msg):
        print("target_num: ", msg.data)
def main(args=None):
    rclpy.init(args=args)
    listener=vision_test()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()