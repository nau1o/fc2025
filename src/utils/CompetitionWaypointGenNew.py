import numpy as np
import mavros_msgs.srv 
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

class CompetitionPoint_test(WayPointShit):
    def __init__(self,home,tar1_gps,tar2_gps,tar3_gps):
        self.home = home
        self.tar1_gps = tar1_gps
        self.tar2_gps = tar2_gps
        self.tar3_gps = tar3_gps
        self.tar1_enu = location.geodetic_to_enu(tar1_gps[0],tar1_gps[1],tar1_gps[2],home[0],home[1],home[2])
        self.tar2_enu = location.geodetic_to_enu(tar2_gps[0],tar2_gps[1],tar2_gps[2],home[0],home[1],home[2])
        self.tar3_enu = location.geodetic_to_enu(tar3_gps[0],tar3_gps[1],tar3_gps[2],home[0],home[1],home[2])

        self.det_ret= self.gen_detect_waypoint()
        self.rally = self.gen_rally_waypoint()
        self.tg1_ret = self.gen_drop_waypoint_v2(self.tar1_enu,1)
        self.tg2_ret = self.gen_drop_waypoint_v2(self.tar2_enu,1)
        self.tg3_ret = self.gen_drop_waypoint_v2(self.tar3_enu,1)
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
        dist32 = np.linalg.norm(tar3_array - tar2_array)
        # 计算垂直向量（2D）
        vertical_v1= self.gen_rotate(np.pi/2) @ v1
        
        # 计算新点时加入高度分量
        st = [*((np.array(ret1[:2]) + 80 * vertical_v1).tolist()), height]
        ed = [*((np.array(ret2[:2]) -30 * v2).tolist()), height]
        subsidiary_point1 = [*((np.array(ret1[:2]) - 15*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(ret1[:2]) + 40*v1 - 15*vertical_v1).tolist()), height]
        subsidiary_point3 = [*((np.array(ret2[:2]) +(dist32/2+40)*v1).tolist()), height]
        
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=10.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi,False,increase=10.)[1:])
        #req.waypoints.extend(self.generate_straight_line_waypoints(ret1, subsidiary_point1, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point2,subsidiary_point3,np.pi,False,increase=10.)[2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,subsidiary_point3, increase=10.)[:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1,subsidiary_point2, increase=10.)[1:-1])
        #req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2,ed, increase=25.)[1:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point3,ret2,np.pi,False,increase=10.)[1:-1])
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
        # 计算目标点2到目标点3的方向向量z
        dir = np.array(self.tar2_enu)[:2] - np.array(self.tar3_enu)[:2]
        dir = dir / np.linalg.norm(dir)  # 单位化
        vet = (np.array(self.tar3_enu)[:2] - np.array(self.tar2_enu)[:2])
        vet = vet / np.linalg.norm(vet)
        vet = self.gen_rotate(np.pi/2) @ vet
        
        bot = self.center_point(self.tar1_enu, self.tar3_enu)[:2] + vet * 30. 
        
        # 计算垂直偏移
        
        
        # 构建3D点并转换回GPS坐标
        rally_point = np.zeros(3)
        rally_point[0] = bot[0]
        rally_point[1] = bot[1]
        rally_point[2] = 20  # 高度设为20米
        
        return location.enu_to_geodetic(*rally_point, *self.home)
    
    def gen_drop_waypoint_v2(self, target: list, idx: int):
        h = target[2]
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        center = np.array(geodetic_to_enu(*self.rally, *self.home))[:2]
        target_2 = np.array(target)[:2]
        #a = np.linalg.norm(target_2 - center)
        a = 70
        # 修改了盘旋半径
        if idx == 1 or idx == 2: b = 60.
        else: b = 70.
        theta = np.arcsin(b/a)
        dir = ((center - target_2) / a)
        dir = gen_rotate(-theta)@ dir
        if idx == 1 or idx == 4: st = 60 * dir + target_2 
        else: st = 80 * dir + target_2
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        if idx == 1 or idx == 4: ed = -60 * dir + target_2 
        else: ed = -40 * dir + target_2
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req