from typing import Tuple

import numpy as np
from matplotlib.path import Path

def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    #res = np.zeros(shape=shape, dtype="float32")
    # these are random values
    #res[100:150, 100:150] = 1
    #res[300:, 200:] = 1
    # ---
    res = get_mask(shape, "left")
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    #res = np.zeros(shape=shape, dtype="float32")
    # these are random values
    #res[100:150, 100:300] = -1
    # ---
    res = get_mask(shape, "right")
    return res


def get_symmetry(P, W):
    Q = []
    for (x,y) in P:
        Q.append((x,W-y))
    return Q

def get_mask(shape: Tuple[int, int], side: str):

    height, width = shape
    x, y = np.mgrid[:height, :width]
    coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1))) 

    # Zone 1: band on left hand side
    p1 = [  (1.0*height, 0.*width), 
            (0.5*height, 0.*width), 
            (0.35*height, 0.25*width), 
            (0.35*height, 0.5*width), 
            (0.7*height, 0.5*width), 
            (0.7*height, 0.26*width),
            (1.*height, 0.02*width)] # start bottom-left corner, going CW
    mask1 = Path(p1).contains_points(coors) * 1
    # Zone2: symmetric to zone1
    p2 = get_symmetry(p1, width)
    mask2 = Path(p2).contains_points(coors) * -1

    # Zone 3: band on left hand side
    p3 = [(1.*height, 0.02*width), (1.*height, 0.5*width), (0.7*height, 0.5*width), (0.7*height, 0.27*width)] # going CCW
    mask3 = Path(p3).contains_points(coors) * 1
    # Zone4: symmetric to zone3
    p4 = get_symmetry(p3, width)
    mask4 = Path(p4).contains_points(coors) * -1

    mask_Left = mask1 + mask2 + mask3 + mask4
    mask_Right = - mask1 - mask2 - mask3 - mask4
    
    if side == "left":
        return mask_Left.reshape(height, width)
    elif side == "right":
        return mask_Right.reshape(height, width)

