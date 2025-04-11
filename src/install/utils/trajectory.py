import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np
import sys
from pose import Quaternion
path = "/home/joe/Desktop/position_pose_velocity.txt"
path1 = "/home/joe/Desktop/expand_position_pose_velocity.txt"

from scipy.spatial.transform import Rotation as R

data = np.genfromtxt(path, dtype=float)
data1 = np.genfromtxt(path1, dtype=float)

fig=plt.figure(figsize=(10,10))
ax1 = Axes3D(fig)
# ax1.plot3D([0,1],[0,1],[0,1], 'red')
x_track = np.zeros((1, 3))
x_track_s = np.array([.0,.0,.0])
idx = 0
def gen_path():
    global idx,x_track,x_track_s
    if idx == len(data): idx = idx-1
    x = data[idx][0]
    y = data[idx][1]
    z = data[idx][2]
    rot = Quaternion(data[idx][3], data[idx][4], data[idx][5], data[idx][6])
    idx+=1
    x_track_s = [x,y,z]
    x_track = np.append(x_track, [x_track_s],axis=0)
    if x_track.size >= 100: x_track=x_track[1:]
    return x_track, rot

def update(i):
    label = 'timestep {0}'.format(i)
    print(label)
    # 更新直线和x轴（用一个新的x轴的标签）。
    # 用元组（Tuple）的形式返回在这一帧要被重新绘图的物体
    x_track, rot = gen_path()
    cur = x_track[-1]
    ori_x, ori_y, ori_z = rot.rotate_vec([40, 0, 0]), rot.rotate_vec([0, 100, 0]), rot.rotate_vec([0, 0, 100])
    axis_x = [[ori_x[0]+cur[0], cur[0]], [ori_x[1]+cur[1], cur[1]] ,[ori_x[2]+cur[2], cur[2]]]
    axis_y = [[ori_y[0]+cur[0], cur[0]], [ori_y[1]+cur[1], cur[1]] ,[ori_y[2]+cur[2], cur[2]]]
    axis_z = [[ori_z[0]+cur[0], cur[0]], [ori_z[1]+cur[1], cur[1]] ,[ori_z[2]+cur[2], cur[2]]]
    # ax1.set_xlabel(label)
    plt.cla()
    ax1.plot3D(x_track[:, 0], x_track[:, 1], x_track[:, 2], 'blue')
    ax1.plot3D(*axis_x, linewidth=3, color='r')
    # ax1.plot3D(*axis_y, linewidth=3, color='b')
    # ax1.plot3D(*axis_z, linewidth=3, color='g')
    # ax1.text(*ori_x, "Front", color='r', va="center", ha="center")
    # ax1.text(*ori_y, "Left", color='b', va="center", ha="center")
    # ax1.text(*ori_z, "Up", color='g', va="center", ha="center")
    ax1.set_xlabel('EAST', fontsize=18)
    ax1.set_ylabel('NORTH', fontsize=18)
    ax1.set_zlabel('UP', fontsize=18)
    # plt.xlim(-1000, 1000)
    # plt.ylim(-1000, 1000)
    # ax1.set_zlim([-1, 500])
    return ax1



if __name__ == '__main__':
    # FuncAnimation 会在每一帧都调用“update” 函数。
    # 在这里设置一个10帧的动画，每帧之间间隔200毫秒
    # print(data.shape)
    # anim = FuncAnimation(fig, update, interval=250)
    # if len(sys.argv) > 1 and sys.argv[1] == 'save':
    #     anim.save('line.gif', dpi=80, writer='imagemagick')
    # else:
    #     plt.show()
    paramL = 600
    paramR = 700
    mul = 4
    data_x = data[paramL:paramR,0]
    data_y = data[paramL:paramR,1]
    data_z = data[paramL:paramR,2]
    ori1_x = data[paramL:paramR,3]
    ori1_y = data[paramL:paramR,4]
    ori1_z = data[paramL:paramR,5]
    ori1_w = data[paramL:paramR,6]
    # print(data[400:410])
    
    data1_x = data1[paramL*mul:paramR*mul,0]
    data1_y = data1[paramL*mul:paramR*mul,1]
    data1_z = data1[paramL*mul:paramR*mul,2]
    ori2_x = data1[paramL*mul:paramR*mul,3]
    ori2_y = data1[paramL*mul:paramR*mul,4]
    ori2_z = data1[paramL*mul:paramR*mul,5]
    ori2_w = data1[paramL*mul:paramR*mul,6]
    
    # for t in range(len(data_x)):
    #     cur = [data_x[t], data_y[t], data_z[t]]
    #     rot = R.from_quat([ori1_x[t], ori1_y[t], ori1_z[t], ori1_w[t]])
    #     ori_x, ori_y, ori_z = rot.apply([10, 0, 0.7]), rot.apply([0, 1, 0]), rot.apply([0, 0, 1])
    #     print(ori_x, ori_y, ori_z)
    #     axis_x = [[ori_x[0]+cur[0], cur[0]], [ori_x[1]+cur[1], cur[1]] ,[ori_x[2]+cur[2], cur[2]]]
    #     axis_y = [[ori_y[0]+cur[0], cur[0]], [ori_y[1]+cur[1], cur[1]] ,[ori_y[2]+cur[2], cur[2]]]
    #     axis_z = [[ori_z[0]+cur[0], cur[0]], [ori_z[1]+cur[1], cur[1]] ,[ori_z[2]+cur[2], cur[2]]]
    #     if (t==0): ax1.plot3D(*axis_x, linewidth=3, color='r')
    #     else: ax1.plot3D(*axis_x, linewidth=3, color='g')
        
        # ax1.plot3D(*axis_y, linewidth=3, color='b')
        # ax1.plot3D(*axis_z, linewidth=3, color='g')
    # print(data1[400*10:410*10])
    # for t in range(len(data1_x)):
    #     cur = [data1_x[t], data1_y[t], data1_z[t]]
    #     rot = R.from_quat([ori2_x[t], ori2_y[t], ori2_z[t], ori2_w[t]])
    #     ori_x, ori_y, ori_z = rot.apply([10, 0, 0.7]), rot.apply([0, 1, 0]), rot.apply([0, 0, 1])
    #     print(ori_x, ori_y, ori_z)
    #     axis_x = [[ori_x[0]+cur[0], cur[0]], [ori_x[1]+cur[1], cur[1]] ,[ori_x[2]+cur[2], cur[2]]]
    #     axis_y = [[ori_y[0]+cur[0], cur[0]], [ori_y[1]+cur[1], cur[1]] ,[ori_y[2]+cur[2], cur[2]]]
    #     axis_z = [[ori_z[0]+cur[0], cur[0]], [ori_z[1]+cur[1], cur[1]] ,[ori_z[2]+cur[2], cur[2]]]
    #     if (t==0): ax1.plot3D(*axis_x, linewidth=3, color='g')
    #     else: ax1.plot3D(*axis_x, linewidth=3, color='g')
    
    ax1.plot3D(data1_x, data1_y, data1_z, linewidth=1, color='r', label="insert")
    ax1.plot3D(data_x, data_y, data_z, linewidth=1, color='blue', label="real")
    plt.show()
    
    lcss = np.zeros((len(data), len(data1)), dtype=float)
    tot = 0.
    print(np.shape(lcss))
    for i in range(1, len(data)):
        pos = np.array([data[i][0], data[i][1], data[i][2]])
        pos1 = np.array([data[i-1][0], data[i-1][1], data[i-1][2]])
        dist = np.linalg.norm(pos - pos1)
        tot += dist
    for i in range(len(data)):
        for j in range(len(data1)):
            pos = np.array([data[i][0], data[i][1], data[i][2]])
            pos1 = np.array([data1[j][0], data1[j][1], data1[j][2]])
            dist = np.linalg.norm(pos - pos1)
            # if np.equal(pos.all(), pos1.all()): lcss[i][j] = 0
            # if dist < 0.1:
            #     if i == 0 or j == 0: lcss[i][j] = 1
            #     else: lcss[i][j] = lcss[i-1][j-1] + 1
            # else:
            #     if i == 0 or j == 0: lcss[i][j] = 0
            #     else: lcss[i][j] = max(lcss[i-1][j], lcss[i][j-1])
            if i==0 or j==0: lcss[i][j] = dist
            else: lcss[i][j] = dist + min(lcss[i-1][j], lcss[i][j-1], lcss[i-1][j-1])
    # coff = float(lcss[len(data)-1][len(data1)-1])/(1. * min(len(data), len(data1)))
    
    print((lcss[len(data)-1][len(data1)-1]) / tot)
# ax = plt.axes(projection = '3d')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_title('3d_mobile_obs')
# ax.set_xlim([-200, 200])
# ax.set_ylim([-200, 200])
# # plt.grid(True)
# plt.ion()  # interactive mode on!!!! 很重要，有了他就不需要plt.show()了

# for t in range(len(data)):
    
#     plt.cla() # 此命令是每次清空画布，所以就不会有前序的效果
#     plt.xlim(-100, 100)
#     plt.ylim(-200, 200)
#     ax.plot3D(x_track[:, 0], x_track[:, 1], x_track[:, 2], 'b')
#     ax.set_zlim([-1, 200])
    
#     x_track = gen_path()
#     plt.pause(0.01)