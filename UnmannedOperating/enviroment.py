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
        self.size_obs = [(6, 15), (10, 40)]
        self.pos_obs = [(-10, 75), (50, 80)]
        self.angle_obs = [pi / 6, 0]
        self.ox, self.oy = [], []
        self.ox_boundary, self.oy_boundary = [], []
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

            for j in range(len):
                self.ox_boundary.append(ReferencePoint[0][0] + int(j * cos(angle)))
                self.oy_boundary.append(ReferencePoint[0][1] + int(j * sin(angle)))
                self.ox_boundary.append(ReferencePoint[1][0] + int(j * cos(angle)))
                self.oy_boundary.append(ReferencePoint[1][1] + int(j * sin(angle)))
            for k in range(width):
                self.ox_boundary.append(ReferencePoint[0][0] - int(k * sin(angle)))
                self.oy_boundary.append(ReferencePoint[0][1] + int(k * cos(angle)))
                self.ox_boundary.append(ReferencePoint[2][0] - int(k * sin(angle)))
                self.oy_boundary.append(ReferencePoint[2][1] + int(k * cos(angle)))
        return
    
    def get_obs_shape(self):
        return self.ox, self.oy

    def get_boundry_shape(self):
        # boundary
        return self.ox_boundary, self.oy_boundary

    def design_obstacles(self):
        ox, oy = [], []

        # obstacle
        for i in range(60):
            ox.append(-25.0)
            oy.append(i)
        for i in range(60):
            ox.append(95.0)
            oy.append(i)
        for i in range(20):
            ox.append(50.0)
            oy.append(30-i)
        for i in range(20):
            ox.append(20.0)
            oy.append(30-i)

        # wall
        for i in range(100):
            ox.append(-50)
            oy.append(i)
        for i in range(100):
            ox.append(150)
            oy.append(i)
        for j in range(-50, 150):
            ox.append(j)
            oy.append(0)
        for j in range(-50, 150):
            ox.append(j)
            oy.append(100)

        return ox, oy
    
    def get_start_pos(self):
        return 60, 10, np.deg2rad(90.0)

    def get_load_pos(self):
        x, y = self.pos_obs[-1][0], self.pos_obs[-1][1]
        initXvalue = 4
        rangex = self.size_obs[-1][1] - initXvalue
        delta = random.randint(initXvalue, rangex)
        return x + delta, y - self.C.RB, np.deg2rad(90.0)

    def get_unload_pos(self):
        len = self.size_obs[0][0]
        beta = self.C.RB
        angle = self.angle_obs[0]
        x, y = self.pos_obs[0][0] + len * cos(angle), self.pos_obs[0][1] + len * sin(angle)
        angle_num = self.angle_obs[0] / pi * 180 + 90
        return x + beta * tan(angle), y - beta, np.deg2rad(angle_num)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    Obs = Obstacle()
    barrierx, barriery = Obs.design_obstacles()
    plt.plot(barrierx, barriery, 'sk')
    plt.show()