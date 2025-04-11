from utils import location
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from utils import classes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np
from utils.classes import BaseNode,CallBackNode
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class straight_line_TestNode(Node):
    def __init__(self):
        BaseNode.__init__(self)
        self.home = [-35.36279571,149.16553531,0]
        self.offboard_setpoint_counter = 0
        self.current_state = 0
        self.local_position_sub = self.create_subscription(Odometry,"/mavros/global_position/local",CallBackNode.local_position_cb,qos_profile)
        self.timer = self.create_timer(0.1, self.timer_cb)
        

    def timer_cb(self):
        if self.local_position is not None:
            
        if self.current_state ==0:
            self.set_mode("AUTO")
            if self.mode_set_success:
                self.get_logger().info("Mode set to AUTO")
                self.current_state = 1
        
        