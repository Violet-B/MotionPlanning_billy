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


class Node:
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct


class Nodes:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []

    def add(self, t, node):
        self.x.append(node.x)
        self.y.append(node.y)
        self.yaw.append(node.yaw)
        self.v.append(node.v)
        self.t.append(t)
        self.direct.append(node.direct)


class PATH:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.ind_end = len(self.cx) - 1
        self.index_old = None

    def target_index(self, node):
        """
        search index of target point in the reference path.
        the distance between target point and current position is ld
        :param node: current information
        :return: index of target point
        """
        if self.index_old is None:
            self.calc_nearest_ind(node)
        Lf = C.kf * node.v + C.Ld
        for ind in range(self.index_old, self.ind_end + 1):
            if self.calc_distance(node, ind) > Lf:
                self.index_old = ind
                return ind, Lf
        self.index_old = self.ind_end

        return self.ind_end, Lf

    def calc_nearest_ind(self, node):
        """
        calc index of the nearest point to current position
        :param node: current information
        :return: index of nearest point
        """
        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        self.index_old = ind

    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])


def pure_pursuit(node, ref_path, index_old):
    """
    pure pursuit controller
    :param node: current information
    :param ref_path: reference path: x, y, yaw, curvature
    :param index_old: target index of last time
    :return: optimal steering angle
    """

    ind, Lf = ref_path.target_index(node)  # target point and pursuit distance
    ind = max(ind, index_old)

    tx = ref_path.cx[ind]
    ty = ref_path.cy[ind]

    alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
    delta = math.atan2(2.0 * C.WB * math.sin(alpha), Lf)

    return delta, ind


def pid_control(target_v, v, dist, direct):
    """
    PID controller and design speed profile.
    :param target_v: target speed (forward and backward are different)
    :param v: current speed
    :param dist: distance from current position to end position
    :param direct: current direction
    :return: desired acceleration
    """

    a = 0.3 * (target_v - direct * v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a


def simulation(x, y, yaw, direct, path_x, path_y):
   return