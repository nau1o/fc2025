import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from quaternion import Quaternion
import time
class PoseVisualizer:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('EAST', fontsize=20)
        self.ax.set_ylabel('NORTH', fontsize=20)
        self.ax.set_zlabel('UP', fontsize=20)
        
    def draw(self, ori_x: list, ori_y: list, ori_z: list) -> None:
        axis_x = [[ori_x[0], 0], [ori_x[1], 0] ,[ori_x[2], 0]]
        axis_y = [[ori_y[0], 0], [ori_y[1], 0] ,[ori_y[2], 0]]
        axis_z = [[ori_z[0], 0], [ori_z[1], 0] ,[ori_z[2], 0]]
        self.ax.plot3D(*axis_x, linewidth=3, color='r')
        self.ax.plot3D(*axis_y, linewidth=3, color='b')
        self.ax.plot3D(*axis_z, linewidth=3, color='g')
        self.ax.text(*ori_x, "Front", color='r', va="center", ha="center")
        self.ax.text(*ori_y, "Left", color='b', va="center", ha="center")
        self.ax.text(*ori_z, "Up", color='g', va="center", ha="center")
        self.ax.set_xlabel('EAST', fontsize=18)
        self.ax.set_ylabel('NORTH', fontsize=18)
        self.ax.set_zlabel('UP', fontsize=18)
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1]) 
    
    def show(self) -> None:
        plt.show()
    
    def set_dynamic(self) -> None:
        plt.ion()
    
    def draw_and_show(self, data) -> None:    
        for t in range(len(data)):
            plt.cla()
            rot = Quaternion(data[t][3], data[t][0], data[t][1], data[t][2])
            self.draw(rot.rotate_vec([1, 0, 0]), rot.rotate_vec([0, 1, 0]), rot.rotate_vec([0, 0, 1]))
            plt.pause(.25)
            # print("YES")
            # self.draw(self, [data[t][0], 0, 0)
# axis_x = [[1, 0], [0, 0] ,[0, 0]]
# axis_y = [[0, 1, 0], [0, 0, 0]]
# axis_z = [[0, 0, 1], [0, 0, 0]]

# x,y,z = np.array([22.5, 50. , 34.6]), np.array([12. , 47.5, 38.4]), np.array([3.5, 7.5, 7.6])
# ax.plot3D(*axis_x)
# plt.show()
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':
    # pose_visualizer = PoseVisualizer()
    # data = np.genfromtxt("/home/joe/Desktop/pose_velocity.txt", dtype=float)

    # last_rot = None
    # inserts = []
    # for rot in data:
    #     rot = Quaternion(rot[6], rot[3], rot[4], rot[5])
    #     if last_rot is not None:
    #         inserts.extend(Quaternion.rotate_insert_slerp(last_rot.rot, rot.rot)[:-1])
    #     last_rot = rot
    #     inserts.append(rot.rot)
    # inserts.append(last_rot.rot)
    
    # insert_data = []
    # for rot in inserts:
    #     tmp = rot.as_quat()
    #     insert_data.append([tmp[0], tmp[1], tmp[2], tmp[3]])
    
    # pose_visualizer.set_dynamic()
    # pose_visualizer.draw_and_show(insert_data)
    
    data = np.genfromtxt("/home/joe/Desktop/position_pose_velocity.txt", dtype=float)
    
    inter = 1/6.
    last_rot = None
    last_pos = np.array([0, 0, 0])
    last_vel = None
    inserts = []
    expand_data = []
    maxn = 0
    errors = []
    for rot in data:
        cur_rot = Quaternion(rot[3], rot[4], rot[5], rot[6])
        cur_pos = np.array([rot[0], rot[1], rot[2]])
        cur_vel = rot[7]
        
        tmp_r = None
        tmp_pos = [last_pos]
        last_pos_cp = last_pos
        tmp_vel = None
        tmp_result = None
        if last_rot is not None:
            tmp_r = (Quaternion.rotate_insert_slerp(last_rot.rot, cur_rot.rot)[:-1])
            tmp_vel = (last_vel + cur_vel) / 2.
            
            expand_data.append([*last_pos, *last_rot.rot.as_quat()])
            for r in tmp_r:
                vec = r.apply([1, 0, 0])
                vec = vec / np.linalg.norm(vec)
                delta = tmp_vel * (inter/5.) * vec
                last_pos_cp = last_pos_cp + delta
                tmp_pos.append(last_pos_cp)
                expand_data.append([*last_pos_cp, *r.as_quat()])
            print (f"上一次位置:[ {last_pos[0], last_pos[1], last_pos[2]} ]")
            print (f"当前位置:[ {cur_pos[0], cur_pos[1], cur_pos[2]} ]")
            
            def getDis(p1, p2, p3):
                p1 = np.array(p1)
                p2 = np.array(p2)
                p3 = np.array(p3)
                vec1 = p1 - p2
                vec2 = p3 - p2
                return np.linalg.norm(np.cross(vec1, vec2)) / np.linalg.norm(vec2)  
                      
            for tt in tmp_pos:
                print (f"插值位置: {tt[0], tt[1], tt[2], getDis(last_pos, cur_pos, tt)}")
                maxn = max(maxn, getDis(last_pos, cur_pos, tt))
                errors.append(getDis(last_pos, cur_pos, tt))
            # time.sleep(1.)
        else: print("init")
        last_rot = cur_rot
        last_vel = cur_vel
        last_pos = cur_pos
        # time.sleep(.01)
    # expand_data.append([*last_pos, *last_rot.rot.as_quat()])
    errors = np.array(errors)
    # print(errors)
    print(f"最大距离：{maxn, errors.mean(), errors.std(), len(tmp_pos)}")
    path1 = "/home/joe/Desktop/expand_position_pose_velocity.txt"
    with open(path1,'w') as f:
        for i in range(len(expand_data)):
            f.write(str(expand_data[i][0])+" "+str(expand_data[i][1])+" "+str(expand_data[i][2])+" "+
                    str(expand_data[i][3])+" "+str(expand_data[i][4])+" "+str(expand_data[i][5])+" "+
                    str(expand_data[i][6])+"\n")
        
            
                