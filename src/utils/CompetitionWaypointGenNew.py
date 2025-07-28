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
import math
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
from utils.classes import BaseNode,CallBackNode,State,WayPointShit,ParameterShit,RallyPointShit
from utils import location

class CompetitionPoint_test(WayPointShit,ParameterShit,BaseNode):
    def __init__(self):
        BaseNode.__init__(self, "CompetitionPoint_test")
        self.alt=50
        
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
        '''

        self.tar1_enu = geodetic_to_enu(self.tar1_gps[0],self.tar1_gps[1],self.tar1_gps[2],self.home[0],self.home[1],self.home[2])
        self.tar2_enu = geodetic_to_enu(self.tar2_gps[0],self.tar2_gps[1],self.tar2_gps[2],self.home[0],self.home[1],self.home[2])
        self.tar3_enu = geodetic_to_enu(self.tar3_gps[0],self.tar3_gps[1],self.tar3_gps[2],self.home[0],self.home[1],self.home[2])
        self.rallypoint = None
        self.rtl_rad = 50.0
        self.chg_parameter("WP_LOITER_RAD",self.rtl_rad)
        self.det_ret= self.gen_detect_waypoint2()
        self.rally = self.gen_rally_waypoint()
        self.drop_wp_ccw1,self.drop_wp_cw1= self.gen_drop_waypoint(self.home[0], self.home[1], self.home[2], self.rally, self.tar1_gps)
        self.drop_wp_ccw2,self.drop_wp_cw2 = self.gen_drop_waypoint(self.home[0], self.home[1], self.home[2], self.rally, self.tar2_gps)
        self.drop_wp_ccw3,self.drop_wp_cw3 = self.gen_drop_waypoint(self.home[0], self.home[1], self.home[2], self.rally, self.tar3_gps)
        
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
        subsidiary_point1 = [*((np.array(ret1[:2]) - 100*vertical_v1).tolist()), height]
        subsidiary_point2 = [*((np.array(self.tar2_enu[:2]) - 100*v3).tolist()), height]
        subsidiary_point3 = [*((np.array(self.tar2_enu[:2]) +50*v3).tolist()), height]

        self.rallypoint = [*((np.array(subsidiary_point3[:2]) + self.rtl_rad*v1).tolist()), height]
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(st,subsidiary_point1, increase=40.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point1,subsidiary_point2,np.pi,False,increase=30.)[0:-1])
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
        
        return enu_to_geodetic(*rally_point, *self.home)
    def gen_drop_waypoint(self, home_lat, home_lon, home_alt, rally_gps, target_gps):
        """生成投弹航线，从集结点圆周的切点飞向投弹点，支持顺时针和逆时针两种切线方向"""
        self.get_logger().info("Generating drop waypoints from tangent point to target (clockwise and counterclockwise)...")
        
        try:
            
            
            # 坐标转换到ENU坐标系
            target_enu = geodetic_to_enu(target_gps[0], target_gps[1], target_gps[2], home_lat, home_lon, home_alt)
            rally_enu = geodetic_to_enu(rally_gps[0], rally_gps[1], rally_gps[2], home_lat, home_lon, home_alt)
            radius = 50.0  # 圆周半径为50米

            # 计算投弹点到集结点的向量和距离
            dx = target_enu[0] - rally_enu[0]
            dy = target_enu[1] - rally_enu[1]
            distance = math.sqrt(dx**2 + dy**2)

            # 检查投弹点是否在圆周内部或过近
            if distance < radius + 1e-6:
                self.get_logger().error(f"Target is inside or too close to rally circle (distance {distance:.2f}m < radius {radius:.2f}m)")
                return (mavros_msgs.srv.WaypointPush.Request(), mavros_msgs.srv.WaypointPush.Request())

            # 计算单位方向向量（从集结点到投弹点）
            dir_x = dx / distance
            dir_y = dy / distance

            # 计算切线角度（sin θ = r/d）
            sin_theta = radius / distance
            theta = math.asin(sin_theta)

            # 初始化航点请求
            req_ccw = mavros_msgs.srv.WaypointPush.Request()  # 逆时针
            req_cw = mavros_msgs.srv.WaypointPush.Request()   # 顺时针
            req_ccw.waypoints = []
            req_cw.waypoints = []

            # 计算切点（基于垂直于集结点到投弹点的向量）
            # 垂直向量（顺时针和逆时针）
            perp_dir_ccw = np.array([-dir_y, dir_x])  # 逆时针旋转90度
            perp_dir_cw = np.array([dir_y, -dir_x])   # 顺时针旋转90度

            # 归一化垂直向量
            perp_dir_ccw = perp_dir_ccw / np.linalg.norm(perp_dir_ccw)
            perp_dir_cw = perp_dir_cw / np.linalg.norm(perp_dir_cw)

            # 计算逆时针切点（起点）
            start_enu_ccw = [
                rally_enu[0] + radius * perp_dir_ccw[0],
                rally_enu[1] + radius * perp_dir_ccw[1],
                target_enu[2] + 10  # 切点高度略高
            ]

            # 计算顺时针切点（起点）
            start_enu_cw = [
                rally_enu[0] + radius * perp_dir_cw[0],
                rally_enu[1] + radius * perp_dir_cw[1],
                target_enu[2] + 10  # 切点高度略高
            ]

            # 终点为投弹点
            end_enu = [
                target_enu[0],
                target_enu[1],
                target_enu[2]  # 保持投弹点高度
            ]


            # 生成逆时针航点
            req_ccw.waypoints.append(self.generate_waypoint(*start_enu_ccw))
            req_ccw.waypoints.extend(self.generate_straight_line_waypoints(start_enu_ccw, end_enu, increase=20.)[:-1])
            self.get_logger().info(f"Generated {len(req_ccw.waypoints)} waypoints for counterclockwise path")

            # 生成顺时针航点
            req_cw.waypoints.append(self.generate_waypoint(*start_enu_cw))
            req_cw.waypoints.extend(self.generate_straight_line_waypoints(start_enu_cw, end_enu, increase=20.)[:-1])
            self.get_logger().info(f"Generated {len(req_cw.waypoints)} waypoints for clockwise path")

            return (req_ccw, req_cw)

        except Exception as e:
            self.get_logger().error(f"Waypoint generation failed: {str(e)}")
            return (mavros_msgs.srv.WaypointPush.Request(), mavros_msgs.srv.WaypointPush.Request())