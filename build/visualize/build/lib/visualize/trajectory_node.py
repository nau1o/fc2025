import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import multiprocessing
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import HomePosition

import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
from utils.location import geodetic_to_enu

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
path1="/home/naynn/Desktop/position_pose_velocity.txt"


class TrajectoryRelateNode(Node):

    def __init__(self):
        super().__init__('pose_relate_node')
        # self.declare_parameter('waypoint_file', "/home/joe/Desktop/waypoints_test/drop/drop6.waypoints")
        self.declare_parameter('waypoint_file', "/home/naynn/Desktop/waypoints_test/drop/drop6.waypoints")
        
        # self.local_position_sub = self.create_subscription(PoseStamped,"/mavros/local_position/pose",self.local_position_cb,qos_profile)
        # self.local_velocity_sub = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.local_velocity_cb, qos_profile)
        # self.global_position_sub = self.create_subscription(NavSatFix, "/mavros/global_position/global",self.global_position_cb, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "/mavros/global_position/local",self.odom_cb, qos_profile)
        self.home_sub = self.create_subscription(HomePosition, "/mavros/home_position/home",self.home_cb, qos_profile)
        
        self.local_data = []
        self.global_data = []
        self.fig = plt.figure(figsize=(5, 5))
        self.trajectory_ax = self.fig.add_subplot(111, projection='3d')
        
        self.x, self.y, self.z, self.w = 0, 0, 0, -10
        self.trajectory = []
        self.pose_draw_timer = self.create_timer(0, self.draw)
        self.home = None
        
        plt.ion()
        
    def home_cb(self, home_position: HomePosition) -> None:
        self.home = [home_position.geo.latitude, home_position.geo.longitude, home_position.geo.altitude]
        print("home: ", self.home)
        
    def draw(self):
        plt.cla()
        minn, maxn = 0, 0
        path = self.get_parameter("waypoint_file").get_parameter_value().string_value
        
        waypoints = []
        flg = False
        if path != "no":
            with open(path, 'r') as f:
                while True:
                    line = f.readline()
                    if not line: break
                    line = line.split()
                    if len(line) < 10: continue
                    waypoints.append([float(line[8]), float(line[9]), float(line[10])])
            home = [self.home[0], self.home[1], 0]
            waypoints = waypoints[1:]
            waypoints_enu = []
            for p in waypoints:
                waypoints_enu.append(geodetic_to_enu(p[0], p[1], p[2], *home))
            waypoints_enu = np.array(waypoints_enu)
            flg = True
            minn = waypoints_enu.min()
            maxn = waypoints_enu.max()
        
        tmp = np.array([[0, 0, 0],[0, 0, 0]])
        if len(self.trajectory) >= 2: 
            tmp = np.array(self.trajectory)
            self.trajectory_ax.plot(tmp[-100:,0],tmp[-100:,1],tmp[-100:,2], color='g', label='global')
            self.trajectory_ax.scatter(tmp[-1,0], tmp[-1, 1], tmp[-1, 2], s=20, color='g')
            
        if len(self.local_data) >= 2:
            tmp = np.array(self.local_data)
            self.trajectory_ax.plot(tmp[-100:,0],tmp[-100:,1],tmp[-100:,2], color='black', label='local')
            self.trajectory_ax.scatter(tmp[-1,0], tmp[-1, 1], tmp[-1, 2], s=20, color='black')
        if flg: 
            self.trajectory_ax.plot(waypoints_enu[:,0], waypoints_enu[:,1], waypoints_enu[:,2], color='r', label='waypoints')
            self.trajectory_ax.scatter(waypoints_enu[:,0], waypoints_enu[:,1], waypoints_enu[:,2], s=15, color='orange')
        minn = min(tmp.min(), minn)
        maxn = max(tmp.max(), maxn)
        self.trajectory_ax.set_xlim([minn, maxn])
        self.trajectory_ax.set_ylim([minn, maxn])
        self.trajectory_ax.set_zlim([minn, maxn])
        plt.legend(loc='best')
        plt.pause(0.01)
        
    def odom_cb(self, odom: Odometry) -> None:
        local_position = odom.pose
        velocity = np.linalg.norm(np.array([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z]))
        self.local_data.append([local_position.pose.position.x,local_position.pose.position.y,local_position.pose.position.z,
                local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w,
                velocity])
    
    def local_position_cb(self, local_position: PoseStamped) -> None:
        self.data.append([local_position.pose.position.x,local_position.pose.position.y,local_position.pose.position.z,
                          local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w,
                          self.velocity])
        self.x, self.y, self.z, self.w = local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w
        # self.trajectory.append([local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z])
        # if len(self.trajectory) > 150: self.trajectory = self.trajectory[1:]
        
    def local_velocity_cb(self, local_velocity: TwistStamped) -> None:
        self.velocity = np.linalg.norm(np.array([local_velocity.twist.linear.x,local_velocity.twist.linear.y,local_velocity.twist.linear.z]))
    
    def global_position_cb(self, global_position: NavSatFix) -> None:
        if self.home is None: return
        tmp = geodetic_to_enu(global_position.latitude, global_position.longitude, global_position.altitude - self.home[2], self.home[0], self.home[1], 0)
        self.trajectory.append(tmp)
        if len(self.trajectory) > 150: self.trajectory = self.trajectory[1:]
    
    def output_file(self):
        print(len(self.data))
        with open(path1,'w') as f:
            for i in range(len(self.data)):
                f.write(str(self.data[i][0])+" "+str(self.data[i][1])+" "+str(self.data[i][2])+" "+
                        str(self.data[i][3])+" "+str(self.data[i][4])+" "+str(self.data[i][5])+" "+
                        str(self.data[i][6])+" "+str(self.data[i][7])+"\n")
                
    def timer_cb(self):
        if len(self.data) > self.size:
            print("没记录完...")
        else:
            self.output_file()
            print("记录完毕")
        self.size = len(self.data)
        
def main(args=None):
    rclpy.init(args=args)
    listener=TrajectoryRelateNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()