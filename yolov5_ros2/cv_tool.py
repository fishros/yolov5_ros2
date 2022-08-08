import cv2
import numpy as np

K = [[602.7175003324863, 0, 351.305582038406],
     [0, 601.6330312976042, 240.0929104708551],
     [0, 0, 1]]
D = [0.06712174262966401, -0.2636999208734844,
     0.006484443443073637, 0.01111161327049835, 0]


def px2xy(point, camera_k, camera_d,  z=1.0):
    MK = np.array(camera_k, dtype=float).reshape(3,3)
    MD = np.array(camera_d, dtype=float)
    point = np.array(point, dtype=float)
    pts_uv = cv2.undistortPoints(point, MK, MD) * z
    return pts_uv[0][0]


# print(px2xy([0, 0], K, D, 1))
