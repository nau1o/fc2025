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



# 外场测试版
class DropWayPointGen(WayPointShit):
    def __init__(self):
        self.home = None
        
        # 从顶点开始逆时针编码, 顺序很重要
        self.tg1_gps = None
        self.tg2_gps = None
        self.tg3_gps = None
        
        self.tg1_enu = None
        self.tg2_enu = None
        self.tg3_enu = None
        self.get_info_from_input()
    
    def __init__(self, home, tg1_gps, tg2_gps, tg3_gps):
        self.home = home
        self.tg1_gps = tg1_gps
        self.tg2_gps = tg2_gps
        self.tg3_gps = tg3_gps
        
        self.tg1_enu = geodetic_to_enu(tg1_gps[0], tg1_gps[1], tg1_gps[2], home[0], home[1], home[2])
        self.tg2_enu = geodetic_to_enu(tg2_gps[0], tg2_gps[1], tg2_gps[2], home[0], home[1], home[2])
        self.tg3_enu = geodetic_to_enu(tg3_gps[0], tg3_gps[1], tg3_gps[2], home[0], home[1], home[2])
        
        self.det_ret = self.gen_detect_waypoint()
        self.output_wp_to_file("/home/naynn/Desktop/way.txt", self.det_ret)
        self.rally = self.gen_rally_waypoint()
        self.tg1_ret = self.gen_drop_waypoint_v2(self.tg1_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way1.txt", self.tg1_ret)
        self.tg2_ret = self.gen_drop_waypoint_v2(self.tg2_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way2.txt", self.tg2_ret)
        self.tg3_ret = self.gen_drop_waypoint_v2(self.tg3_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way3.txt", self.tg3_ret)
    
    def get_info_from_input(self):
        print("home坐标的高度alt请全部输入为0, 请确保三个标靶组成一个三角形, 且编号从角度最大的顶点开始 !!!")
        home = input("请输入home点经纬度(格式: lat lon alt): ")
        home = home.split(" ")
        for i in range(len(home)):
            home[i] = float(home[i])
        print(f'home点: {home[0], home[1], home[2]}')
        self.home = home
        
        print("标靶从顶点开始逆时针编码, 顺序很重要, 标靶坐标的高度alt决定了整条侦察航线的高度 !!!")
        tg1 = input("请输入tg1点经纬度(格式: lat lon alt): ")
        tg1 = tg1.split(" ")
        for i in range(len(tg1)):
            tg1[i] = float(tg1[i])
        print(f'tg1点: {tg1[0], tg1[1], tg1[2]}')
        self.tg1_gps = tg1

        tg2 = input("请输入tg2点经纬度(格式: lat lon alt): ") 
        tg2 = tg2.split(" ")
        for i in range(len(tg2)):
            tg2[i] = float(tg2[i])
        print(f'tg2点: {tg2[0], tg2[1], tg2[2]}')
        self.tg2_gps = tg2
        
        tg3 = input("请输入tg3点经纬度(格式: lat lon alt): ")
        tg3 = tg3.split(" ")
        for i in range(len(tg3)):
            tg3[i] = float(tg3[i])
        print(f'tg3点: {tg3[0], tg3[1], tg3[2]}')
        self.tg3_gps = tg3
        
        self.tg1_enu = geodetic_to_enu(*self.tg1_gps, *self.home)
        self.tg2_enu = geodetic_to_enu(*self.tg2_gps, *self.home)
        self.tg3_enu = geodetic_to_enu(*self.tg3_gps, *self.home)
    
        # 生成侦察航线
        self.ret = self.gen_detect_waypoint()
        self.output_wp_to_file("/home/naynn/Desktop/way.txt", self.ret)
        
        # 生成投弹航线
        self.ret = self.gen_drop_waypoint(self.tg1_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way1.txt", self.ret)
        
        # 生成投弹航线
        self.ret = self.gen_drop_waypoint(self.tg2_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way2.txt", self.ret)
        
        # 生成投弹航线
        self.ret = self.gen_drop_waypoint(self.tg3_enu)
        self.output_wp_to_file("/home/naynn/Desktop/way3.txt", self.ret)
        
    def get_subsidiary_point_loc(self, st:list, ed:list, dist:float) -> list:
        st = np.array(st)
        ed = np.array(ed)
        
        v = (ed - st) / np.linalg.norm(ed - st)
        ret = (ed + dist * v).tolist()
        return ret
    def get_angle(self, top, bot1, bot2) -> float:
        
        top = np.array(top)
        bot1 = np.array(bot1)
        bot2 = np.array(bot2)
        vec1 = bot1 - top
        vec2 = bot2 - top
        return (np.arccos(vec1.dot(vec2) / (np.linalg.norm(vec1)*np.linalg.norm(vec2))) / np.pi) * 180
    
    def gen_detect_waypoint(self):
        subsidiary_point1 = self.get_subsidiary_point_loc(self.tg1_enu, self.tg2_enu, 80)
        subsidiary_point2 = self.get_subsidiary_point_loc(self.tg2_enu, self.tg1_enu, 80)
        
        subsidiary_point3 = self.get_subsidiary_point_loc(self.tg1_enu, self.tg3_enu, 80)
        subsidiary_point4 = self.get_subsidiary_point_loc(self.tg3_enu, self.tg1_enu, 80)
        
        tmp = 180. - self.get_angle(self.tg1_enu, self.tg2_enu, self.tg3_enu)
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point3, subsidiary_point4, increase=25.))
        tmp = self.generate_curve_line_waypoints_radius(subsidiary_point2, subsidiary_point4, 50, False, 20.)[::-1]
        req.waypoints.extend(tmp[2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point2, subsidiary_point1, increase=25.))
        return req
    
    # 比赛四个点天井点坐标生成侦察航线
    
    def output_wp_to_file(self, path:str, req: mavros_msgs.srv.WaypointPush.Request):
        with open(path, 'w') as f:
            f.write("QGC WPL 110\n")
            for i in range(len(req.waypoints)):
                f.write(f"{i}\t{0}\t{req.waypoints[i].frame}\t{req.waypoints[i].command}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req.waypoints[i].x_lat}\t{req.waypoints[i].y_long}\t{req.waypoints[i].z_alt}\t{1}\n")
    
    # 不会根据盘旋点作出变化            
    def gen_drop_waypoint(self, target: list):
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        dir = np.array(self.tg2_enu) - np.array(self.tg3_enu)
        tmp = (dir / np.linalg.norm(dir))[:2]
        rot = gen_rotate(np.pi / 2) @ tmp
        dir[0] = rot[0]
        dir[1] = rot[1]
        st = dir * 150. + np.array(target)
        ed = dir * -40. + np.array(target)
        req.waypoints.extend(self.generate_straight_line_waypoints(st, ed, increase=25.))
        return req
    
    # 从盘旋点开始出发
    # TODO: 针对性修改了一些东西
    def gen_drop_waypoint_v2(self, target: list):
        h = target[2]
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        center = np.array(geodetic_to_enu(*self.rally, *self.home))[:2]
        target_2 = np.array(target)[:2]
        a = np.linalg.norm(target_2 - center)
        
        # 修改了盘旋半径
        b = 50.
        theta = np.arcsin(b/a)
        dir = ((center - target_2) / a)
        dir = gen_rotate(theta)@ dir
        st = 160 * dir + target_2 
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        ed = -60 * dir + target_2 
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req
        
    # TODO: 针对性修改了一些东西
    def gen_rally_waypoint(self):
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        dir = np.array(self.tg2_enu) - np.array(self.tg3_enu)
        mid = (np.array(self.tg2_enu) + np.array(self.tg1_enu)) / 2
        tmp = (dir / np.linalg.norm(dir))[:2]
        rot = gen_rotate(np.pi / 2) @ tmp
        dir[0] = rot[0]
        dir[1] = rot[1]
        st = (dir * 250. + np.array(mid)).tolist()
        
        # 修改的部分
        # st[0] = 30
        st[-1] = 40
        st = enu_to_geodetic(*st, *self.home)
        return st
    



    
