import os
import sys
import math
import heapq
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial.kdtree as kd
from algorithm import astar
import algorithm.reeds_shepp as rs
from config import ParaConfig
from units import *

C = ParaConfig()

class Node:
    def __init__(self, xind, yind, yawind, direction, x, y,
                 yaw, yawt, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yawt = yawt
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind


class Para:
    def __init__(self, minx, miny, minyaw, minyawt, maxx, maxy, maxyaw, maxyawt,
                 xw, yw, yaww, yawtw, xyreso, yawreso, ox, oy, kdtree):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.minyawt = minyawt
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.maxyawt = maxyawt
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.yawtw = yawtw
        self.xyreso = xyreso
        self.yawreso = yawreso
        self.ox = ox
        self.oy = oy
        self.kdtree = kdtree


class Path:
    def __init__(self, x, y, yaw, yawt, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yawt = yawt
        self.direction = direction
        self.cost = cost


class QueuePrior:
    def __init__(self):
        self.queue = []

    def empty(self):
        return len(self.queue) == 0  # if Q is empty

    def put(self, item, priority):
        heapq.heappush(self.queue, (priority, item))  # reorder x using priority

    def get(self):
        return heapq.heappop(self.queue)[1]  # pop out element with smallest priority


def hybrid_astar_planning(sx, sy, syaw, syawt, gx, gy,
                          gyaw, gyawt, ox, oy, xyreso, yawreso):
    """
    planning hybrid A* path.
    :param sx: starting node x position [m]
    :param sy: starting node y position [m]
    :param syaw: starting node yaw angle [rad]
    :param syawt: starting node trailer yaw angle [rad]
    :param gx: goal node x position [m]
    :param gy: goal node y position [m]
    :param gyaw: goal node yaw angle [rad]
    :param gyawt: goal node trailer yaw angle [rad]
    :param ox: obstacle x positions [m]
    :param oy: obstacle y positions [m]
    :param xyreso: grid resolution [m]
    :param yawreso: yaw resolution [m]
    :return: hybrid A* path
    """

    sxr, syr = round(sx / xyreso), round(sy / xyreso)
    gxr, gyr = round(gx / xyreso), round(gy / xyreso)
    syawr = round(rs.pi_2_pi(syaw) / yawreso)
    gyawr = round(rs.pi_2_pi(gyaw) / yawreso)

    nstart = Node(sxr, syr, syawr, 1, [sx], [sy], [syaw], [syawt], [1], 0.0, 0.0, -1)
    ngoal = Node(gxr, gyr, gyawr, 1, [gx], [gy], [gyaw], [gyawt], [1], 0.0, 0.0, -1)

    kdtree = kd.KDTree([[x, y] for x, y in zip(ox, oy)])
    P = calc_parameters(ox, oy, xyreso, yawreso, kdtree)

    hmap = astar.calc_holonomic_heuristic_with_obstacle(ngoal, P.ox, P.oy, P.xyreso, 1.0)
    steer_set, direc_set = calc_motion_set()
    open_set, closed_set = {calc_index(nstart, P): nstart}, {}

    qp = QueuePrior()
    qp.put(calc_index(nstart, P), calc_hybrid_cost(nstart, hmap, P))

    while True:
        if not open_set:
            return None

        ind = qp.get()
        n_curr = open_set[ind]
        closed_set[ind] = n_curr
        open_set.pop(ind)

        update, fpath = update_node_with_analystic_expantion(n_curr, ngoal, gyawt, P)

        if update:
            fnode = fpath
            break

        yawt0 = n_curr.yawt[0]

        for i in range(len(steer_set)):
            node = calc_next_node(n_curr, ind, steer_set[i], direc_set[i], P)

            if not is_index_ok(node, yawt0, P):
                continue

            node_ind = calc_index(node, P)

            if node_ind in closed_set:
                continue

            if node_ind not in open_set:
                open_set[node_ind] = node
                qp.put(node_ind, calc_hybrid_cost(node, hmap, P))
            else:
                if open_set[node_ind].cost > node.cost:
                    open_set[node_ind] = node

    print("final expand node: ", len(open_set) + len(closed_set))

    return extract_path(closed_set, fnode, nstart)


def extract_path(closed, ngoal, nstart):
    rx, ry, ryaw, ryawt, direc = [], [], [], [], []
    cost = 0.0
    node = ngoal

    while True:
        rx += node.x[::-1]
        ry += node.y[::-1]
        ryaw += node.yaw[::-1]
        ryawt += node.yawt[::-1]
        direc += node.directions[::-1]
        cost += node.cost

        if is_same_grid(node, nstart):
            break

        node = closed[node.pind]

    rx = rx[::-1]
    ry = ry[::-1]
    ryaw = ryaw[::-1]
    ryawt = ryawt[::-1]
    direc = direc[::-1]

    direc[0] = direc[1]
    path = Path(rx, ry, ryaw, ryawt, direc, cost)

    return path


def update_node_with_analystic_expantion(n_curr, ngoal, gyawt, P):
    path = analystic_expantion(n_curr, ngoal, P)  # rs path: n -> ngoal

    if not path:
        return False, None

    steps = [C.MOVE_STEP * d for d in path.directions]
    yawt = calc_trailer_yaw(path.yaw, n_curr.yawt[-1], steps)

    if abs(rs.pi_2_pi(yawt[-1] - gyawt)) >= C.GOAL_YAW_ERROR:
        return False, None

    fx = path.x[1:-1]
    fy = path.y[1:-1]
    fyaw = path.yaw[1:-1]

    fd = []
    for d in path.directions[1:-1]:
        if d >= 0:
            fd.append(1.0)
        else:
            fd.append(-1.0)
    # fd = path.directions[1:-1]

    fcost = n_curr.cost + calc_rs_path_cost(path, yawt)
    fpind = calc_index(n_curr, P)
    fyawt = yawt[1:-1]
    fsteer = 0.0

    fpath = Node(n_curr.xind, n_curr.yind, n_curr.yawind, n_curr.direction,
                 fx, fy, fyaw, fyawt, fd, fsteer, fcost, fpind)

    return True, fpath


def analystic_expantion(node, ngoal, P):
    sx, sy, syaw = node.x[-1], node.y[-1], node.yaw[-1]
    gx, gy, gyaw = ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1]

    maxc = math.tan(C.MAX_STEER) / C.WB
    paths = rs.calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=C.MOVE_STEP)

    if not paths:
        return None

    pq = QueuePrior()
    for path in paths:
        steps = [C.MOVE_STEP * d for d in path.directions]
        yawt = calc_trailer_yaw(path.yaw, node.yawt[-1], steps)
        pq.put(path, calc_rs_path_cost(path, yawt))

    # while not pq.empty():
    path = pq.get()
    steps = [C.MOVE_STEP * d for d in path.directions]
    yawt = calc_trailer_yaw(path.yaw, node.yawt[-1], steps)
    ind = range(0, len(path.x), C.COLLISION_CHECK_STEP)

    pathx = [path.x[k] for k in ind]
    pathy = [path.y[k] for k in ind]
    pathyaw = [path.yaw[k] for k in ind]
    pathyawt = [yawt[k] for k in ind]

    if not is_collision(pathx, pathy, pathyaw, pathyawt, P):
        return path

    return None


def calc_next_node(n, ind, u, d, P):
    step = C.XY_RESO * 2.0

    nlist = math.ceil(step / C.MOVE_STEP)
    xlist = [n.x[-1] + d * C.MOVE_STEP * math.cos(n.yaw[-1])]
    ylist = [n.y[-1] + d * C.MOVE_STEP * math.sin(n.yaw[-1])]
    yawlist = [rs.pi_2_pi(n.yaw[-1] + d * C.MOVE_STEP / C.WB * math.tan(u))]
    yawtlist = [rs.pi_2_pi(n.yawt[-1] +
                           d * C.MOVE_STEP / C.RTR * math.sin(n.yaw[-1] - n.yawt[-1]))]

    for i in range(nlist - 1):
        xlist.append(xlist[i] + d * C.MOVE_STEP * math.cos(yawlist[i]))
        ylist.append(ylist[i] + d * C.MOVE_STEP * math.sin(yawlist[i]))
        yawlist.append(rs.pi_2_pi(yawlist[i] + d * C.MOVE_STEP / C.WB * math.tan(u)))
        yawtlist.append(rs.pi_2_pi(yawtlist[i] +
                                   d * C.MOVE_STEP / C.RTR * math.sin(yawlist[i] - yawtlist[i])))

    xind = round(xlist[-1] / P.xyreso)
    yind = round(ylist[-1] / P.xyreso)
    yawind = round(yawlist[-1] / P.yawreso)

    cost = 0.0

    if d > 0:
        direction = 1.0
        cost += abs(step)
    else:
        direction = -1.0
        cost += abs(step) * C.BACKWARD_COST

    if direction != n.direction:  # switch back penalty
        cost += C.GEAR_COST

    cost += C.STEER_ANGLE_COST * abs(u)  # steer penalyty
    cost += C.STEER_CHANGE_COST * abs(n.steer - u)  # steer change penalty
    cost += C.SCISSORS_COST * sum([abs(rs.pi_2_pi(x - y))
                                   for x, y in zip(yawlist, yawtlist)])  # jacknif cost
    cost = n.cost + cost

    directions = [direction for _ in range(len(xlist))]

    node = Node(xind, yind, yawind, direction, xlist, ylist,
                yawlist, yawtlist, directions, u, cost, ind)

    return node


def is_collision(x, y, yaw, yawt, P):
    for ix, iy, iyaw, iyawt in zip(x, y, yaw, yawt):
        d = 0.5
        deltal = (C.RTF - C.RTB) / 2.0
        rt = (C.RTF + C.RTB) / 2.0 + d

        ctx = ix + deltal * math.cos(iyawt)
        cty = iy + deltal * math.sin(iyawt)

        idst = P.kdtree.query_ball_point([ctx, cty], rt)

        if idst:
            for i in idst:
                xot = P.ox[i] - ctx
                yot = P.oy[i] - cty

                dx_trail = xot * math.cos(iyawt) + yot * math.sin(iyawt)
                dy_trail = -xot * math.sin(iyawt) + yot * math.cos(iyawt)

                if abs(dx_trail) <= rt and \
                        abs(dy_trail) <= C.W / 2.0 + d:
                    return True

        deltal = (C.RF - C.RB) / 2.0
        rc = (C.RF + C.RB) / 2.0 + d

        cx = ix + deltal * math.cos(iyaw)
        cy = iy + deltal * math.sin(iyaw)

        ids = P.kdtree.query_ball_point([cx, cy], rc)

        if ids:
            for i in ids:
                xo = P.ox[i] - cx
                yo = P.oy[i] - cy

                dx_car = xo * math.cos(iyaw) + yo * math.sin(iyaw)
                dy_car = -xo * math.sin(iyaw) + yo * math.cos(iyaw)

                if abs(dx_car) <= rc and \
                        abs(dy_car) <= C.W / 2.0 + d:
                    return True
        #
        # theta = np.linspace(0, 2 * np.pi, 200)
        # x1 = ctx + np.cos(theta) * rt
        # y1 = cty + np.sin(theta) * rt
        # x2 = cx + np.cos(theta) * rc
        # y2 = cy + np.sin(theta) * rc
        #
        # plt.plot(x1, y1, 'b')
        # plt.plot(x2, y2, 'g')

    return False


def calc_trailer_yaw(yaw, yawt0, steps):
    yawt = [0.0 for _ in range(len(yaw))]
    yawt[0] = yawt0

    for i in range(1, len(yaw)):
        yawt[i] += yawt[i - 1] + steps[i - 1] / C.RTR * math.sin(yaw[i - 1] - yawt[i - 1])

    return yawt


def trailer_motion_model(x, y, yaw, yawt, D, d, L, delta):
    x += D * math.cos(yaw)
    y += D * math.sin(yaw)
    yaw += D / L * math.tan(delta)
    yawt += D / d * math.sin(yaw - yawt)

    return x, y, yaw, yawt


def calc_rs_path_cost(rspath, yawt):
    cost = 0.0

    for lr in rspath.lengths:
        if lr >= 0:
            cost += 1
        else:
            cost += abs(lr) * C.BACKWARD_COST

    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:
            cost += C.GEAR_COST

    for ctype in rspath.ctypes:
        if ctype != "S":
            cost += C.STEER_ANGLE_COST * abs(C.MAX_STEER)

    nctypes = len(rspath.ctypes)
    ulist = [0.0 for _ in range(nctypes)]

    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = -C.MAX_STEER
        elif rspath.ctypes[i] == "WB":
            ulist[i] = C.MAX_STEER

    for i in range(nctypes - 1):
        cost += C.STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    cost += C.SCISSORS_COST * sum([abs(rs.pi_2_pi(x - y))
                                   for x, y in zip(rspath.yaw, yawt)])

    return cost


def calc_motion_set():
    s = [i for i in np.arange(C.MAX_STEER / C.N_STEER,
                              C.MAX_STEER, C.MAX_STEER / C.N_STEER)]

    steer = [0.0] + s + [-i for i in s]
    direc = [1.0 for _ in range(len(steer))] + [-1.0 for _ in range(len(steer))]
    steer = steer + steer

    return steer, direc


def calc_hybrid_cost(node, hmap, P):
    cost = node.cost + \
           C.H_COST * hmap[node.xind - P.minx][node.yind - P.miny]

    return cost


def calc_index(node, P):
    ind = (node.yawind - P.minyaw) * P.xw * P.yw + \
          (node.yind - P.miny) * P.xw + \
          (node.xind - P.minx)

    yawt_ind = round(node.yawt[-1] / P.yawreso)
    ind += (yawt_ind - P.minyawt) * P.xw * P.yw * P.yaww

    return ind


def is_index_ok(node, yawt0, P):
    if node.xind <= P.minx or \
            node.xind >= P.maxx or \
            node.yind <= P.miny or \
            node.yind >= P.maxy:
        return False

    steps = [C.MOVE_STEP * d for d in node.directions]
    yawt = calc_trailer_yaw(node.yaw, yawt0, steps)

    ind = range(0, len(node.x), C.COLLISION_CHECK_STEP)

    x = [node.x[k] for k in ind]
    y = [node.y[k] for k in ind]
    yaw = [node.yaw[k] for k in ind]
    yawt = [yawt[k] for k in ind]

    if is_collision(x, y, yaw, yawt, P):
        return False

    return True


def calc_parameters(ox, oy, xyreso, yawreso, kdtree):
    minxm = min(ox) - C.EXTEND_AREA
    minym = min(oy) - C.EXTEND_AREA
    maxxm = max(ox) + C.EXTEND_AREA
    maxym = max(oy) + C.EXTEND_AREA

    ox.append(minxm)
    oy.append(minym)
    ox.append(maxxm)
    oy.append(maxym)

    minx = round(minxm / xyreso)
    miny = round(minym / xyreso)
    maxx = round(maxxm / xyreso)
    maxy = round(maxym / xyreso)

    xw, yw = maxx - minx, maxy - miny

    minyaw = round(-C.PI / yawreso) - 1
    maxyaw = round(C.PI / yawreso)
    yaww = maxyaw - minyaw

    minyawt, maxyawt, yawtw = minyaw, maxyaw, yaww

    P = Para(minx, miny, minyaw, minyawt, maxx, maxy, maxyaw,
             maxyawt, xw, yw, yaww, yawtw, xyreso, yawreso, ox, oy, kdtree)

    return P


def is_same_grid(node1, node2):
    if node1.xind != node2.xind or \
            node1.yind != node2.yind or \
            node1.yawind != node2.yawind:
        return False

    return True