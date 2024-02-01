"""
Pure Pursuit
author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt


class Nodes:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []


class PATH:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.ind_end = len(self.cx) - 1
        self.index_old = None
