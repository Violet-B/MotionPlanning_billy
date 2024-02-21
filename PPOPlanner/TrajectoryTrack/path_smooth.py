"""
Pure Pursuit For RL
authors: Wang Ning; Yang Hongyi
Date: 2024-02-01
"""

import os
import sys
import math
import numpy as np
from os import path
import matplotlib.pyplot as plt
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import TrajectoryTrack.reeds_shepp as rs
from CarConfig.pid_config import C


def generate_path(s):
    """
    Generate the path using Reeds_Shepp.
    
    Args:
        s: List of states [(x, y, yaw), ...]
        
    Returns:
        path_x: List of x-coordinates of the path
        path_y: List of y-coordinates of the path
        yaw: List of yaw angles of the path
        direct: List of directions of the path
        x_all: List of all x-coordinates of the path
        y_all: List of all y-coordinates of the path
    """
    max_c = math.tan(C.MAX_STEER) / C.WB  # max curvature

    path_x, path_y, yaw, direct = [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []
    direct_flag = 1.0

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])
        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw,
                                      g_x, g_y, g_yaw, 
                                      max_c)
        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions

        for j in range(len(ix)):
            if idirect[j] == direct_flag:
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                x_rec, y_rec, yaw_rec, direct_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]], [-direct_rec[-1]]

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)

    x_all, y_all = [], []

    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, direct, x_all, y_all