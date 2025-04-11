import numpy as np
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
class Quaternion:
    def __init__(self, x, y, z, w):
        self.rot = R.from_quat([x, y, z, w])
    def rotate_vec(self, vec: list):
        return self.rot.apply(vec)
    
    def rotate_insert_slerp(quat1: R, quat2: R, interval = 0.25):
        slerp = Slerp([0, 1], R.from_quat([quat1.as_quat(), quat2.as_quat()]))
        times = np.arange(0, 1, interval)
        return slerp(times)

if __name__ == "__main__":

    # q = Quaternion(-0.2674060100525621,
    #                -0.10960170500795806,
    #                 -0.5805649464376619,
    #               -0.7612002594685265
    #                )
    
    # # p = R.from_quat([0, 0, 0, 1])
    # print((q.rot.inv().as_matrix()).dot([1, 0, 0])) 
    # print(q.rotate_vec([1, 0, 0]))
    with open("/home/joe/Desktop/wp2.waypoints", "r") as f:
        while True:
            line = f.readline()
            if not line: break
            line = line.split()
            