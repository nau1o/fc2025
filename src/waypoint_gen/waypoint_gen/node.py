import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import nav_msgs.msg
from rcl_interfaces.msg import SetParametersResult
import rclpy
import mavros_msgs
from rclpy.node import Node
import geometry_msgs
import sensor_msgs
import sensor_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import numpy as np
import nav_msgs
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
from utils.classes import BaseNode, WayPointShit, TakeOffShit, ParameterShit, RallyPointShit, CallBackNode, State
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.DropWayPointGen import DropWayPointGen
from utils.CompeitionWaypointA import DropWayPointGenA

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MainNode(WayPointShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self)
        self.new_mission = True
        # self.timer = self.create_timer(0.1, self.timer_cb2)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.home = [22.5905687, 113.9750004, 0]
        self.offboard_setpoint_counter = 0
        self.chg = False
        self.cache = None
        self.control_state = State.CLEAR_WP
            
    def gen_drop_waypoint(self):

        self.get_logger().info("生成中...")
        req = mavros_msgs.srv.WaypointPush.Request()
        # start = geodetic_to_enu(22.59094024, 113.97535239, 120, *self.home)
        # end = geodetic_to_enu(22.58933996, 113.97537561, 120, *self.home)
        # req.waypoints.append(self.generate_waypoint(*start))
        # req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=20.))
        # self.waypoint_push(req)
        
        start = geodetic_to_enu(22.59094219, 113.97505543 , 120, *self.home)
        end = geodetic_to_enu(22.59004532, 113.97505278 , 120, *self.home)
        req.waypoints.append(self.generate_waypoint(*start))
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.))
        start = end
        end = geodetic_to_enu(22.590042, 113.975831 , 120, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints(start, end, 180.*np.pi/180., False, increase=20.)[1:])
        # self.waypoint_push(req)
        start = geodetic_to_enu(22.590042, 113.975831 , 120, *self.home)
        end = geodetic_to_enu(22.590950, 113.975838, 110, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
            
        req = mavros_msgs.srv.WaypointPush.Request()
        start = geodetic_to_enu (22.590950, 113.975838, 110, *self.home)
        end = geodetic_to_enu (22.591887, 113.976163, 90, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu (22.591887,113.976163, 90, *self.home)
        end = geodetic_to_enu (22.591917, 113.975300, 75, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints_radius(start, end, 48., False, 20.)[:-1])
        
        start = geodetic_to_enu (22.5919174000, 113.9752996000, 75, *self.home)
        end = geodetic_to_enu (22.591014, 113.975315, 50, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu (22.591014, 113.975315, 50, *self.home)
        end = geodetic_to_enu (22.590112, 113.975331, 50, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:])
        
        start = geodetic_to_enu (22.590112, 113.975331, 50, *self.home)
        end = geodetic_to_enu ( 22.5893637000, 113.9753522000, 90, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu ( 22.5893637000, 113.9753522000, 90, *self.home)
        end = geodetic_to_enu (22.58975071, 113.97595540, 130, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints_radius(start, end, 40., False, 20)[:])
        
        return req.waypoints
    def timer_cb(self):
        # print("working...")
        # print(self.control_state)
        if self.offboard_setpoint_counter <= 20:
            req = mavros_msgs.srv.WaypointPush.Request()
            start = geodetic_to_enu (22.59121603, 113.97535472, 120, *self.home)
            end = geodetic_to_enu (22.59032007, 113.97535472, 120, *self.home)
            req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=20.)[:])
            self.cache = req
            
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
                self.waypoint_push(self.cache)
        
        elif self.control_state == State.CLEAR_RALLY:
            if self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == True:
                self.rallypoint_clear_success == False
                self.control_state = State.PUSH_RALLY
            elif self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == False:
                self.rallypoint_new_clear_req = True
                self.rallypoint_clear()
        
        elif self.control_state == State.PUSH_RALLY:
            if self.state.mode != "AUTO": return
            if self.rallypoint_new_push_req == False and self.rallypoint_push_success == True:
                self.rallypoint_push_success == False
                self.control_state = State.CHG_PARAM
            elif self.rallypoint_new_push_req == False and self.rallypoint_push_success == False:
                self.rallypoint_new_push_req = True
                self.rallypoint_push([22.58997037, 113.97536734, 120.])
                
        
        # if (
        #     self.current_reached_waypoint == self.current_waypoint_num
        #     and self.current_reached_waypoint != None
        #     and self.new_mission == True
        # ):
        #     print("mission complete!")
        #     self.new_mission = False
        
        
        self.offboard_setpoint_counter += 1
        
    def timer_cb2(self):
        if self.chg == True or self.offboard_setpoint_counter < 10: 
            self.offboard_setpoint_counter += 1
            return
        if self.parameter_pull_success == False :
            self.pull_parameter(True)
        if self.parameter_pull_success == True and self.chg == False: 
            self.rallypoint_pull()
            self.chg = True
        self.offboard_setpoint_counter += 1
        # if self.home != None:
        #     print(self.home)
        
        
        
class PlayGroundNode(WayPointShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self)
        self.new_mission = True
        # self.timer = self.create_timer(0.1, self.timer_cb2)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.home = [22.5905687, 113.9750004, 0.]
        self.offboard_setpoint_counter = 0
        self.chg = False
        self.cache = None
        self.control_state = State.CLEAR_WP
        
    def timer_cb(self):
        # print("working...")
        # print(self.control_state)
        if self.offboard_setpoint_counter <= 20:
            tmp = DropWayPointGenA([38.55836766, 115.14099924, 0], [38.55957692, 115.14290759, 15], [38.55971915, 115.14313070, 15], [38.55986327, 115.14298034, 15], [38.55970967, 115.14275965, 15])
            # print(tmp.rally)
            self.cache = tmp.det_ret
            
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
                self.waypoint_push(self.cache)
        
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
                tmp = DropWayPointGenA([38.55836766, 115.14099924, 0], [38.55957692, 115.14290759, 15], [38.55971915, 115.14313070, 15], [38.55986327, 115.14298034, 15], [38.55970967, 115.14275965, 15])
                self.rallypoint_push(tmp.rally)
                
        
        self.offboard_setpoint_counter += 1
            
def main(args=None):
    rclpy.init(args=args)
    node = PlayGroundNode()
    rclpy.spin(node)
    # print("spin")
    node.destroy_node()
    rclpy.shutdown()