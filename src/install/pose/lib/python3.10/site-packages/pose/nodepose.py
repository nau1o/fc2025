import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
import std_msgs.msg
from utils.classes import BaseNode, WayPointShit, TakeOffShit, ParameterShit, RallyPointShit, CallBackNode, State
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.DropWayPointGen import DropWayPointGen
from utils.CompetitionWaypointGenNew import CompetitionPoint_test
import std_msgs
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
path1="/home/mocha/Desktop/pose_velocity.txt"
wp_path = "waypoint/way"

'''
def init_wp_info():
    ret = []
    for i in range(1,5):
        with open(wp_path+str(i)+".txt") as f:
            lines = []
            for line in f.readlines():
                wp = line.split("\t")
                lines.append(wp)
            ret.append([[float(lines[2][8]), float(lines[2][9]), float(lines[2][10])], len(lines)-2])
    return ret
'''
def check_vector(vec1, vec2):
    return vec1.dot(vec2) < 0

class SwitchToAutoNode(WayPointShit, ParameterShit, RallyPointShit,BaseNode):
    def __init__(self):
        BaseNode.__init__(self,node_name="pose_node")
        # 订阅标靶信息
        self.target_sub = self.create_subscription(std_msgs.msg.UInt32, "/vision/target_num", self.target_num_cb, qos_profile)
        #self.tg_wp_info = init_wp_info()
        self.final_drop_first_wp = []
        self.final_drop_wp_num = None
        self.detect_wp_num = 26
        self.final_target = None
        self.stage = -1
        # 关于集结点盘旋的判断
        tmp = CompetitionPoint_test()
        self.rally_point_gps = tmp.rally
        self.rally_point_enu = []
        self.rally_hover_flg = False
        self.rally_hover_counter = 0
        self.rally_hover_log = False
        #self.drop1 = tmp.drop_wp_ccw1#顺时针
        self.drop1 = tmp.drop_wp_cw1 #逆时针
        self.drop2 = tmp.drop_wp_ccw2
        self.drop3 = tmp.drop_wp_ccw3
        # 关于切换的判断
        self.switch_to_auto_log_flg = False
        self.wp_push_success_flg = 0
        self.wp_push_success_log_flg = False
        self.mission_fin = False
        self.pull_parameter(True)
        self.FFFinal_target = None
        self.param_chg_timer = self.create_timer(0.1, self.target_get_timer_cb)
        self.mode_timer = self.create_timer(0.1, self.mode_get_timer_cb)
        self.control_state = 0


    def gps_to_enu_cb(self):
        if self.home == None: return
        self.rally_point_enu = geodetic_to_enu(self.rally_point_gps[0], self.rally_point_gps[1], self.rally_point_gps[2], *self.home)
    def mode_get_timer_cb(self):
        if self.control_state == 0:
            if self.state.mode == 'AUTO':
                print(self.state.mode)
            elif self.state.mode == 'RTL':
                print(self.state.mode)
                if self.final_target == None:
                    self.get_logger().info("未获取到目标标靶")
                    self.FFFinal_target = input("请输入目标标靶编号: ")
                elif self.final_target != None:
                    self.get_logger().info("获取到目标标靶: " + str(self.final_target))
                    self.FFFinal_target = self.final_target
                self.control_state = State.CLEAR_WP

        if self.control_state == State.CLEAR_WP:
            if self.waypoint_new_clear_req == False and self.waypoint_clear_success == True:
                self.waypoint_clear_success == False
                self.control_state = State.PUSH_WP
                print(2)
            elif self.waypoint_new_clear_req == False and self.waypoint_clear_success == False:
                self.waypoint_new_clear_req = True
                self.waypoint_clear()
                print(1)
        
        elif self.control_state == State.PUSH_WP:
            if self.waypoint_new_push_req == False and self.waypoint_push_success == True:
                self.waypoint_push_success == False
                self.control_state = State.CLEAR_RALLY
            elif self.waypoint_new_push_req == False and self.waypoint_push_success == False:
                #self.waypoint_push(self.drop1)
                self.waypoint_new_push_req = True
                print(self.FFFinal_target)
                if self.FFFinal_target == 1:
                    print(5)
                    self.waypoint_push(self.drop1)
                elif self.FFFinal_target == 2:
                    self.waypoint_push(self.drop2)
                elif self.FFFinal_target == 3:   
                    self.waypoint_push(self.drop3)
                '''
                if self.final_target ==1:
                    print(5)
                    #self.waypoint_push(self.drop1)
                    print(4)
                elif self.final_target ==2:
                    self.waypoint_push(self.drop2)
                elif self.final_target ==3:   
                    self.waypoint_push(self.drop3)
                '''


    def target_get_timer_cb(self):
        if self.stage == 0 and self.state.mode=='RTL':self.stage = 1
        if self.stage == 1:
            if self.parameter_new_chg_req == False and self.parameter_chg_success == True:
                self.parameter_chg_success = False
                self.stage = 2
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter("TARGET_NUM", float(self.final_target))
        
        elif self.stage == 2:
            if self.parameters_new_get_req == False and self.parameters_get_success == True:
                self.parameter_chg_success = False
                self.stage = 4
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameters_new_get_req = True
                self.get_parameters(["TARGET_AUTO"])
                if(self.parameters_get_success == True):
                    self.get_logger().info("获取到参数: " + str(self.parameters_get_ret))

        elif self.stage == 4:
            print("获取到目标标靶: " + str(self.parameters_get_ret))
            if(self.paramters_get_ret[0].double_value == 1):
                self.get_logger().info("切换到AUTO模式")
                self.state_new_mode_req = True
                self.state_new_mode = "AUTO"
                self.state_new_mode_cb()
                self.stage = 5
            else:
                self.get_logger().info("未切换到AUTO模式")

    
    
    def target_num_cb(self, target_num: std_msgs.msg.UInt32) -> None:
        if self.stage == -1:
            self.final_target = target_num.data
            self.get_logger().info("获取到目标标靶: " + str(self.final_target))
            #self.final_drop_first_wp, self.final_drop_wp_num = self.tg_wp_info[self.final_target - 1]
            self.stage = 0

    
def main(args=None):
    rclpy.init(args=args)
    listener=SwitchToAutoNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()