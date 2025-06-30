import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np

import mavros_msgs
import mavros_msgs.srv
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
from utils.location import geodetic_to_enu
from utils.classes import BaseNode,CallBackNode,State,WayPointShit,ParameterShit,RallyPointShit
from utils import location
from utils.CompetitionWaypointGenNew import CompetitionPoint_test
class TestNode(WayPointShit, ParameterShit, RallyPointShit,BaseNode):
    def __init__(self):
        BaseNode.__init__(self,node_name="test_node")
        self.new_mission = True
        self.home = [22.5905424,113.9749189,0]
        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.state = None
        self.target1_gps =[22.59060325,113.97553417,50]
        self.target2_gps =[22.59042208,113.97542204,50]
        self.target3_gps =[22.59041920,113.97564942 ,50]
        self.chg = False
        self.wp_det = None
        self.wp_drop = None
        self.rally = None
        self.control_state = State.CLEAR_WP
        self.target = input("请输入目标点号(1,2,3): ")
        self.state_Next = 1#测试用

    def timer_cb(self):
        
            
        if self.offboard_setpoint_counter <= 20:
            tmp = CompetitionPoint_test()
            # print(tmp.rally)
            self.wp_det = tmp.det_ret
            self.rally = tmp.rally
            print(self.state)
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
                self.waypoint_push(self.wp_det)
                self.state_Next = 2
                
        
        elif self.control_state == State.CLEAR_RALLY:
            if self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == True:
                self.rallypoint_clear_success == False
                self.control_state = State.PUSH_RALLY
            elif self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == False:
                self.rallypoint_new_clear_req = True
                self.rallypoint_clear()
        
        elif self.control_state == State.PUSH_RALLY:
            # if self.state.mode != "AUTO": return
            if self.rallypoint_new_push_req == False and self.rallypoint_push_success == True:
                self.rallypoint_push_success == False
                self.control_state = State.CHG_PARAM
            elif self.rallypoint_new_push_req == False and self.rallypoint_push_success == False:
                self.rallypoint_new_push_req = True
                #tmp = DropWayPointGenA([38.55836766, 115.14099924, 0], [38.55957692, 115.14290759, 15], [38.55971915, 115.14313070, 15], [38.55986327, 115.14298034, 15], [38.55970967, 115.14275965, 15])
                self.rallypoint_push(self.rally)
                print(self.rally)
                
                self.state_Next = 3
        if self.state_Next == 3:
            return 0
                
               
        self.offboard_setpoint_counter += 1
        print(self.offboard_setpoint_counter)
         
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