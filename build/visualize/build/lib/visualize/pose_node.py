import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import multiprocessing
from scipy.spatial.transform import Rotation as R

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
path1="/home/joe/Desktop/pose_velocity1.txt"


class PoseRelateNode(Node):

    def __init__(self):
        super().__init__('pose_relate_node')
        self.local_position_sub = self.create_subscription(Odometry,"/mavros/global_position/local",self.local_position_cb,qos_profile)
        self.local_velocity_sub = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.local_velocity_cb, qos_profile)
        self.data = []
        self.velocity = []
        self.timer = self.create_timer(10, self.timer_cb)
        self.size = 0
        self.fig = plt.figure(figsize=(5, 5))
        self.pose_ax = self.fig.add_subplot(111, projection='3d')
        
        self.x, self.y, self.z, self.w = 0, 0, 0, -10
        self.trajectory = []
        self.pose_draw_timer = self.create_timer(0, self.draw)
        # self.trajectory_draw_timer = self.create_timer(0, self.draw_trajectory)
        
        plt.ion()

        
    def draw(self):
        plt.cla()
        # if (self)
        if (self.w == -10):
            ori_x, ori_y, ori_z = [1, 0, 0], [0, 1, 0], [0, 0, 1]
            axis_x = [[0, 1], [0, 0], [0, 0]]
            axis_y = [[0, 0], [0, 1], [0, 0]]
            axis_z = [[0, 0], [0, 0], [0, 1]]
        else:
            rot = R.from_quat([self.x, self.y, self.z, self.w])
            ori_x, ori_y, ori_z = rot.apply([1, 0, 0]), rot.apply([0, 1, 0]), rot.apply([0, 0, 1])
            axis_x = [[0, ori_x[0]], [0, ori_x[1]] ,[0, ori_x[2]]]
            axis_y = [[0, ori_y[0]], [0, ori_y[1]] ,[0, ori_y[2]]]
            axis_z = [[0, ori_z[0]], [0, ori_z[1]] ,[0, ori_z[2]]]
            
        self.pose_ax.plot3D(*axis_x, linewidth=3, color='r')
        self.pose_ax.plot3D(*axis_y, linewidth=3, color='b')
        self.pose_ax.plot3D(*axis_z, linewidth=3, color='g')
        self.pose_ax.text(*ori_x, "front", color='r', va="center", ha="center", fontsize=20)
        self.pose_ax.text(*ori_y, "left", color='b', va="center", ha="center", fontsize=20)
        self.pose_ax.text(*ori_z, "up", color='g', va="center", ha="center", fontsize=20)
        self.pose_ax.set_xlim([-1, 1])
        self.pose_ax.set_ylim([-1, 1])
        self.pose_ax.set_zlim([-1, 1])
        plt.pause(0.01)
        
        
    
    def local_position_cb(self, local_position: Odometry) -> None:
        # self.data.append([local_position.pose.position.x,local_position.pose.position.y,local_position.pose.position.z,
        #                   local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w,
        #                   self.velocity])
        local_position = local_position.pose
        self.x, self.y, self.z, self.w = local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w
        self.trajectory.append([local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z])
        if len(self.trajectory) > 1000: self.trajectory = self.trajectory[1:]
        
    def local_velocity_cb(self, local_velocity: TwistStamped) -> None:
        self.velocity = np.linalg.norm(np.array([local_velocity.twist.linear.x,local_velocity.twist.linear.y,local_velocity.twist.linear.z]))
        
    def output_file(self):
        with open(path1,'w') as f:
            for i in range(len(self.data)):
                f.write(str(self.data[i][0])+" "+str(self.data[i][1])+" "+str(self.data[i][2])+" "+
                        str(self.data[i][3])+" "+str(self.data[i][4])+" "+str(self.data[i][5])+" "+
                        str(self.data[i][6])+" "+str(self.data[i][7])+"\n")
                
    def timer_cb(self):
        if len(self.data) > self.size:
            print("没记录完...")
        else:
            # self.output_file()
            print("记录完毕")
        self.size = len(self.data)
        
def main(args=None):
    rclpy.init(args=args)
    listener=PoseRelateNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()