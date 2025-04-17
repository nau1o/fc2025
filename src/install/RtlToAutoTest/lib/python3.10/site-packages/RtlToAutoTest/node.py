import time
import sys
import os
import rclpy
import numpy as np
import mavros_msgs.srv
import mavros_msgs.msg
from rclpy.node import Node
from pathlib import Path
from utils.CompetitionWaypointGenNew import CompetitionPoint_test
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
from utils.classes import BaseNode, WayPointShit, TakeOffShit, ParameterShit, RallyPointShit, CallBackNode, State
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.DropWayPointGen import DropWayPointGen
from utils.classes import BaseNode,CallBackNode,State,WayPointShit,ParameterShit,RallyPointShit
from utils import location
from utils.CompeitionWaypointA import DropWayPointGenA
class TestNode(BaseNode,State,WayPointShit,ParameterShit,RallyPointShit):
    def __init__(self):
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.rally = None
        self.control_state = State.CLEAR_WP
        self.target = input("请输入目标点号(1,2,3): ")
        self.mode_now = self.state.mode


    def timer_cb(self):
        
            
        if self.mode_now == "RTL":

            tmp = CompetitionPoint_test(self.home, self.target1_gps, self.target2_gps, self.target3_gps)
            # print(tmp.rally)
            self.wp_det = tmp.det_ret
            if self.target == "1":
                self.wp_drop = tmp.tg1_ret
            elif self.target == "2":
                self.wp_drop = tmp.tg2_ret
            elif self.target == "3":
                self.wp_drop = tmp.tg3_ret
            self.rally = tmp.rally
        elif self.control_state == State.CLEAR_WP:
            if self.waypoint_new_clear_req == False and self.waypoint_clear_success == True:
                self.waypoint_clear_success == False
                self.control_state = State.PUSH_WP
            elif self.waypoint_new_clear_req == False and self.waypoint_clear_success == False:
                self.waypoint_new_clear_req = True
                self.waypoint_clear()
        elif self.control_state == State.PUSH_WP:
            if self.waypoint_new_push_req == False and self.waypoint_push_success == True:
                self.waypoint_push_success == False
                self.control_state = State.CLEAR_RALLY
            elif self.waypoint_new_push_req == False and self.waypoint_push_success == False:
                self.waypoint_new_push_req = True
                self.waypoint_push(self.wp_drop)
                
            # print(self.wp_det)
            # print(self.wp_drop)
            # print(self.rally)
            # print(tmp.rally)
            # print(tmp.det_ret)
            # print(tmp.tg1_ret)
            # print(tmp.tg2_ret)
            # print(tmp.tg3_ret)


def main(args=None):
    try:
        # 初始化 ROS2
        rclpy.init(args=args)
        
        # 创建节点
        node = TestNode()
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        # 清理资源
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
