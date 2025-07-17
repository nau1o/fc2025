import numpy as np
import rcl_interfaces.msg
import rcl_interfaces.srv
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
import mavros_msgs.msg
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.classes import WayPointShit
import mavros_msgs
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

class CompetitionPoint_test(WayPointShit,ParameterShit,BaseNode):
    def __init__(self):
        BaseNode.__init__(self, "CompetitionPoint_test")
        self.alt=50
        '''
        #操场
        self.home = [22.5905424,113.9749189,0]#操场
        self.tar1_gps =[22.59060325,113.97553417,50]
        self.tar2_gps =[22.59042208,113.97542204,50]
        self.tar3_gps =[22.59041920,113.97564942 ,50]
        '''
        self.home = [22.80230855,114.29545009,0 ]#外场
        self.tar1_gps =[22.8026997,114.2958453,50]
        self.tar2_gps =[22.8025046,114.2957587,50]
        self.tar3_gps =[22.8025135,114.2959447,50]
        

        self.tar1_enu = location.geodetic_to_enu(self.tar1_gps[0],self.tar1_gps[1],self.tar1_gps[2],self.home[0],self.home[1],self.home[2])
        self.tar2_enu = location.geodetic_to_enu(self.tar2_gps[0],self.tar2_gps[1],self.tar2_gps[2],self.home[0],self.home[1],self.home[2])
        self.tar3_enu = location.geodetic_to_enu(self.tar3_gps[0],self.tar3_gps[1],self.tar3_gps[2],self.home[0],self.home[1],self.home[2])
        self.rallypoint = None
        self.rtl_rad = 50.0
        self.chg_parameter("WP_LOITER_RAD",-self.rtl_rad)
        self.det_ret= self.gen_detect_waypoint4()
        self.rally = self.gen_rally_waypoint()
        
    def center_point(self, point1:list, point2:list):
        # 返回三维坐标的中点，包括高度
        return [
            (point1[0] + point2[0])/2,  # x坐标中点
            (point1[1] + point2[1])/2,  # y坐标中点
            (point1[2] + point2[2])/2   # z坐标中点
        ]
    def gen_rotate(self,angle):
        return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    #侦察路线1
    
    def gen_detect_waypoint1(self):
        # 获取目标点的高度
        height = self.tar1_enu[2]  # 使用目标点1的高度作为基准
        
        # 计算中点
        ret1 = self.center_point(self.tar1_enu, self.tar2_enu)
        ret2 = self.center_point(self.tar1_enu, self.tar3_enu)
        
        # 转换为numpy数组，只取xy平面坐标
        tar1_array = np.array(self.tar1_enu[:2])
        tar2_array = np.array(self.tar2_enu[:2])
        tar3_array = np.array(self.tar3_enu[:2])
        
        # 计算单位向量（2D）
        v1 = (tar2_array - tar3_array) / np.linalg.norm(tar2_array - tar3_array)
        v2 = (tar2_array - tar1_array) / np.linalg.norm(tar2_array - tar1_array)
        
        dist12 = np.linalg.norm(tar2_array - tar1_array)
        dist32 = np.linalg.norm(tar3_array - tar2_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        
        # 计算新点时加入高度分量
        st = [*((np.array(ret1[:2]) + 80 * vertical_v1).tolist()), height]
        ed = [*((np.array(ret2[:2]) -30 * v2).tolist()), height]
        subsidiary_point1 = [*((np.array(ret1[:2]) - 15*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(ret2[:2]) - 15*vertical_v1).tolist()), height]
        subsidiary_point3 = [*((np.array(ret2[:2]) +(dist32/2+40)*v1).tolist()), height]
        subsidiary_point3 = [*((np.array(ret2[:2]) +(dist32/2+40)*v1).tolist()), height]
    # 增加rallypoint赋值，避免NoneType错误
        self.rallypoint = subsidiary_point3

        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=20.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi,False,increase=20.)[1:])
        #req.waypoints.extend(self.generate_straight_line_waypoints(ret1, subsidiary_point1, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point2,subsidiary_point3,np.pi,False,increase=10.)[2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,subsidiary_point3, increase=20.)[:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1,subsidiary_point2, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed, increase=25.)[1:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point3,ret2,np.pi,False,increase=20.)[1:-1])
        return req
    def gen_detect_waypoint2(self):
        # 获取目标点的高度
        height = self.tar1_enu[2]  # 使用目标点1的高度作为基准
        
        # 计算中点
        ret1 = self.center_point(self.tar1_enu, self.tar3_enu)
        ret2 = self.center_point(self.tar1_enu, self.tar2_enu)
        
        # 转换为numpy数组，只取xy平面坐标
        tar1_array = np.array(self.tar1_enu[:2])
        tar2_array = np.array(self.tar2_enu[:2])
        tar3_array = np.array(self.tar3_enu[:2])
        
        # 计算单位向量（2D）
        v1 = (tar2_array - tar3_array) / np.linalg.norm(tar2_array - tar3_array)
        v2 = (tar2_array - tar1_array) / np.linalg.norm(tar2_array - tar1_array)
        
        dist12 = np.linalg.norm(tar2_array - tar1_array)
        dist32 = np.linalg.norm(tar3_array - tar2_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        
        # 计算新点时加入高度分量
        st = [*((np.array(ret1[:2]) + 80 * vertical_v1).tolist()), height]
        ed = [*((np.array(ret2[:2]) -30 * v2).tolist()), height]
        subsidiary_point1 = [*((np.array(ret1[:2]) - 25*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(ret1[:2]) + 50*v1 - 25*vertical_v1).tolist()), height]
        subsidiary_point3 = [*((np.array(ret2[:2]) +40*v1+50*vertical_v1).tolist()), height]
        subsidiary_point4 = [*((np.array(ret2[:2]) + 50*vertical_v1).tolist()), height]
        subsidiary_point5 = [*((np.array(ret2[:2]) - 150*vertical_v1).tolist()), height]
        self.rallypoint = [*((np.array(subsidiary_point5[:2]) + self.rtl_rad*v1).tolist()), height+10]
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=40.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi,False,increase=30.)[0:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(ret1, subsidiary_point1, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point2,subsidiary_point3,np.pi,False,increase=10.)[2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,subsidiary_point3, increase=40.)[:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1,subsidiary_point2, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed, increase=25.)[1:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point3,subsidiary_point4,np.pi,False,increase=30.)[0:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point4,ret2,increase=40.)[1:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(ret2,subsidiary_point5,increase=40.)[1:])
        return req
    def gen_detect_waypoint3(self):
        # 获取目标点的高度
        height = self.tar1_enu[2]  # 使用目标点1的高度作为基准
        
        # 计算中点
        ret1 = self.center_point(self.tar1_enu, self.tar3_enu)
        ret2 = self.center_point(self.tar1_enu, self.tar2_enu)
        
        # 转换为numpy数组，只取xy平面坐标
        tar1_array = np.array(self.tar1_enu[:2])
        tar2_array = np.array(self.tar2_enu[:2])
        tar3_array = np.array(self.tar3_enu[:2])
        
        # 计算单位向量（2D）
        v1 = (tar2_array - tar3_array) / np.linalg.norm(tar2_array - tar3_array)
        v2 = (tar2_array - tar1_array) / np.linalg.norm(tar2_array - tar1_array)
        v3 = (tar3_array - tar1_array) / np.linalg.norm(tar3_array - tar1_array)
        
        dist12 = np.linalg.norm(tar2_array - tar1_array)
        dist32 = np.linalg.norm(tar3_array - tar2_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        
        # 计算新点时加入高度分量
        st = [*((np.array(ret1[:2]) + 80 * vertical_v1).tolist()), height]
        ed = [*((np.array(ret2[:2]) -30 * v2).tolist()), height]
        subsidiary_point1 = [*((np.array(ret1[:2]) - 60*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(self.tar2_enu[:2]) - 60*v3).tolist()), height]
        subsidiary_point3 = [*((np.array(self.tar2_enu[:2]) +50*v3).tolist()), height]

        self.rallypoint = [*((np.array(subsidiary_point3[:2]) + self.rtl_rad*v1).tolist()), height]
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=40.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi/0.8,False,increase=30.)[0:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(ret1, subsidiary_point1, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point2,subsidiary_point3,np.pi,False,increase=10.)[2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,subsidiary_point3, increase=40.)[:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1,subsidiary_point2, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed, increase=25.)[1:-1])
        return req
    def gen_detect_waypoint4(self):
        # 获取目标点的高度
        height = self.tar1_enu[2]  # 使用目标点1的高度作为基准
        
        # 计算中点
        ret1 = self.center_point(self.tar1_enu, self.tar3_enu)
        ret2 = self.center_point(self.tar1_enu, self.tar2_enu)
        
        # 转换为numpy数组，只取xy平面坐标
        tar1_array = np.array(self.tar1_enu[:2])
        tar2_array = np.array(self.tar2_enu[:2])
        tar3_array = np.array(self.tar3_enu[:2])
        
        # 计算单位向量（2D）
        v1 = (tar1_array - tar3_array) / np.linalg.norm(tar1_array - tar3_array)
        v2 = (tar1_array - tar2_array) / np.linalg.norm(tar1_array - tar2_array)
        
        dist12 = np.linalg.norm(tar2_array - tar1_array)
        dist32 = np.linalg.norm(tar3_array - tar2_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        vertical_v2= self.gen_rotate(np.pi/2) @ v2 
        # 计算新点时加入高度分量
        st = [*((np.array(self.tar3_enu[:2]) - 80 * v1).tolist()), height]
        ed = [*((np.array(self.tar2_enu[:2]) -50 * vertical_v2).tolist()), height]
        subsidiary_point1 = [*((np.array(self.tar1_enu[:2]) + 80*v1).tolist()), height]
        subsidiary_point2 = [*((np.array(self.tar2_enu[:2]) +50*vertical_v2).tolist()), height]
        self.rallypoint = [*((np.array(ed[:2]) - self.rtl_rad*v2).tolist()), height]
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=40.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi*0.8,False,increase=40.)[0:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed,increase=40.)[1:])
        return req
    '''
    #侦察路线2
    def gen_detect_waypoint(self):
        # 获取目标点的高度
        height = self.tar1_enu[2]  # 使用目标点1的高度作为基准
        
        # 计算中点
        ret1 = self.center_point(self.tar1_enu, self.tar2_enu)
        ret2 = self.center_point(self.tar1_enu, self.tar3_enu)
        
        # 转换为numpy数组，只取xy平面坐标
        tar1_array = np.array(self.tar1_enu[:2])
        tar2_array = np.array(self.tar2_enu[:2])
        tar3_array = np.array(self.tar3_enu[:2])
        
        # 计算单位向量（2D）
        v1 = (tar2_array - tar3_array) / np.linalg.norm(tar2_array - tar3_array)
        v2 = (tar2_array - tar1_array) / np.linalg.norm(tar2_array - tar1_array)
        
        dist12 = np.linalg.norm(tar2_array - tar1_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        # 计算新点时加入高度分量
        st = [*((np.array(ret1[:2]) + 80 * vertical_v1).tolist()), height]
        ed = [*((np.array(ret2[:2]) -30 * v2).tolist()), height]
        subsidiary_point1 = [*((np.array(ret1[:2]) + 30*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(ret1[:2]) + 15*v1).tolist()), height]
        subsidiary_point3 = [*((np.array(subsidiary_point2[:2]) + 30*vertical_v1).tolist()), height]
        subsidiary_point4 = [*((np.array(ret2[:2]) + 30*vertical_v1).tolist()), height]
        ret1 = [*((np.array(ret1[:2]) - 15*vertical_v1).tolist()), height]
        ret2 = [*((np.array(ret2[:2]) - 15*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(subsidiary_point2[:2]) - 15*vertical_v1).tolist()), height]
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=10.)[:-1])
        #req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,ret1,np.pi,True,increase=10.)[::-1][2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1,ret1, increase=10.)[1:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(ret1,subsidiary_point2,np.pi,False,increase=8.))
        #req.waypoints.extend(self.generate_straight_line_waypoints(ret1,subsidiary_point2, increase=10.)[1:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,subsidiary_point3, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed, increase=25.)[1:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point3,subsidiary_point4,np.pi,False,increase=10.)[1:])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point3,subsidiary_point4, increase=3.)[1:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point4,ret2, increase=10.)[1:-1])
        return req
    '''
    
    def gen_rally_waypoint(self):
        """生成集结点"""

        rally_point = np.zeros(3)
        rally_point[0] = self.rallypoint[0]
        rally_point[1] = self.rallypoint[1]
        rally_point[2] = self.rallypoint[2]
        
        return location.enu_to_geodetic(*rally_point, *self.home)