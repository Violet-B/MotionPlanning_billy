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
from Utils import draw


class Node:
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct
        
    def update(self, a, delta, direct):
        # delta = self.limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.direct = direct
        self.v += self.direct * a * C.dt

    @staticmethod
    def limit_input(delta):
        if delta > 1.2 * C.MAX_STEER:
            return 1.2 * C.MAX_STEER

        if delta < -1.2 * C.MAX_STEER:
            return -1.2 * C.MAX_STEER

        return delta


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



def CarControl(x, y, yaw, direct, path_x, path_y):
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = x[0][0], y[0][0], yaw[0][0], direct[0][0]
    x_rec, y_rec = [], []

    for cx, cy, cyaw, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
        nodes = Nodes()
        nodes.add(t, node)
        ref_trajectory = PATH(cx, cy)
        target_ind, _ = ref_trajectory.target_index(node)

        while t <= C.maxTime:
            if cdirect[0] > 0:
                target_speed = 30.0 / 3.6
                C.Ld = 4.0
                C.dist_stop = 1.5
                C.dc = -1.1
            else:
                target_speed = 20.0 / 3.6
                C.Ld = 2.5
                C.dist_stop = 0.2
                C.dc = 0.2

            xt = node.x + C.dc * math.cos(node.yaw)
            yt = node.y + C.dc * math.sin(node.yaw)
            dist = math.hypot(xt - cx[-1], yt - cy[-1])

            if dist < C.dist_stop:
                break

            acceleration = pid_control(target_speed, node.v, dist, cdirect[0])
            delta, target_ind = pure_pursuit(node, ref_trajectory, target_ind)

            t += C.dt

            node.update(acceleration, delta, cdirect[0])
            nodes.add(t, node)
            x_rec.append(node.x)
            y_rec.append(node.y)

            dy = (node.yaw - yaw_old) / (node.v * C.dt)
            steer = rs.pi_2_pi(-math.atan(C.WB * dy))

            yaw_old = node.yaw
            x0 = nodes.x[-1]
            y0 = nodes.y[-1]
            yaw0 = nodes.yaw[-1]
            direct0 = nodes.direct[-1]

            # animation
            plt.cla()
            plt.plot(node.x, node.y, marker='.', color='k')
            plt.plot(path_x, path_y, color='gray', linewidth=2)
            plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
            plt.plot(cx[target_ind], cy[target_ind], ".r")
            draw.draw_car(node.x, node.y, yaw_old, steer, C)

            # for m in range(len(states)):
            #     draw.Arrow(states[m][0], states[m][1], np.deg2rad(states[m][2]), 2, 'blue')

            plt.axis("equal")
            plt.title("PurePursuit: v=" + str(node.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()



def DrawPath(path_x, path_y):
    plt.cla()
    plt.plot(path_x, path_y, color='gray', linewidth=2)
    plt.axis('equal')
    plt.show()