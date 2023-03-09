import random
import numpy as np
from math import sin, cos, tan,pi
from numpy import arange
from config import ParaConfig


class Obstacle:
    """
    Generate Obs Shape
    """
    ox, oy = [], []
    
    def __init__(self) -> None:
        # init obs
        # index 0: unloading area
        # index 0~-1: obstacles 
        # index -1: loading area
        self.C = ParaConfig()
        self.num_obs = 2
        self.size_obs = [(6, 15), (10, 20)]
        self.pos_obs = [(-10, 75), (70, 80)]
        self.angle_obs = [pi / 6, 0]
        self.init_obs_shape()
        return 
    
    def init_obs_shape(self):
        for i in range(self.num_obs):
            # obs shape 
            width = self.size_obs[i][0]
            len = self.size_obs[i][1]
            x = self.pos_obs[i][0]
            y = self.pos_obs[i][1]
            angle = self.angle_obs[i]
            # four points of obs 
            ReferencePoint = [
                (x, y), 
                (x - width * sin(angle), y + width * cos(angle)),
                (x + len * cos(angle), y + len * sin(angle))]
            # collection of obs shape points
            for j in arange(0, len, 0.1):
                self.ox.append(ReferencePoint[0][0] + j * cos(angle))
                self.oy.append(ReferencePoint[0][1] + j * sin(angle))
                self.ox.append(ReferencePoint[1][0] + j * cos(angle))
                self.oy.append(ReferencePoint[1][1] + j * sin(angle))
            for k in arange(0, width, 0.1):
                self.ox.append(ReferencePoint[0][0] - k * sin(angle))
                self.oy.append(ReferencePoint[0][1] + k * cos(angle))
                self.ox.append(ReferencePoint[2][0] - k * sin(angle))
                self.oy.append(ReferencePoint[2][1] + k * cos(angle))
        return
    
    def get_obs_shape(self):
        ox, oy = self.design_obstacles()
        return self.ox + ox, self.oy + oy

    def design_obstacles(self):
        ox, oy = [], []
        
        for j in range(0, 30):
            ox.append(6)
            oy.append(j)
        for j in range(0, 30):
            ox.append(30)
            oy.append(j)
        for i in range(6, 31):
            ox.append(i)
            oy.append(30)

        return ox, oy
    
    def get_start_pos(self):
        return 60, 10, np.deg2rad(90.0), np.deg2rad(90.0)

    def get_load_pos(self):
        x, y = self.pos_obs[-1][0], self.pos_obs[-1][1]
        initXvalue = 4
        rangex = self.size_obs[-1][1] - initXvalue
        delta = random.randint(initXvalue, rangex)
        return x + delta, y - self.C.RTB, np.deg2rad(90.0), np.deg2rad(90.0)

    def get_unload_pos(self):
        len = self.size_obs[0][0]
        beta = self.C.RTB
        angle = self.angle_obs[0]
        x, y = self.pos_obs[0][0] + len * cos(angle), self.pos_obs[0][1] + len * sin(angle)
        angle_num = self.angle_obs[0] / pi * 180 + 90
        return x + beta * tan(angle), y - beta, np.deg2rad(angle_num), np.deg2rad(angle_num)
