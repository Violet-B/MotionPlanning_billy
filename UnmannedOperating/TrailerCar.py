import numpy as np
import matplotlib.pyplot as plt
from math import atan, sin, cos, pi
from numpy import arange
from config import ParaConfig


class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.3 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + pi - angle
        theta_hat_R = theta + pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L],
                 [y_hat_start, y_hat_end_L], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R],
                 [y_hat_start, y_hat_end_R], color=c, linewidth=w)


class TrailerCar:
    """
    Generate Car Shape
    """
    def __init__(self) -> None:
        return
    
    def draw_model(self, x, y, yaw, yawt, steer, color='black'):
        C = ParaConfig()
        car = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                    [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

        trail = np.array([[-C.RTB, -C.RTB, C.RTF, C.RTF, -C.RTB],
                        [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

        wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                        [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])

        rlWheel = wheel.copy()
        rrWheel = wheel.copy()
        frWheel = wheel.copy()
        flWheel = wheel.copy()
        rltWheel = wheel.copy()
        rrtWheel = wheel.copy()

        Rot1 = np.array([[cos(yaw), -sin(yaw)],
                        [sin(yaw), cos(yaw)]])

        Rot2 = np.array([[cos(steer), -sin(steer)],
                        [sin(steer), cos(steer)]])

        Rot3 = np.array([[cos(yawt), -sin(yawt)],
                        [sin(yawt), cos(yawt)]])

        frWheel = np.dot(Rot2, frWheel)
        flWheel = np.dot(Rot2, flWheel)

        frWheel += np.array([[C.WB], [-C.WD / 2]])
        flWheel += np.array([[C.WB], [C.WD / 2]])
        rrWheel[1, :] -= C.WD / 2
        rlWheel[1, :] += C.WD / 2

        frWheel = np.dot(Rot1, frWheel)
        flWheel = np.dot(Rot1, flWheel)

        rrWheel = np.dot(Rot1, rrWheel)
        rlWheel = np.dot(Rot1, rlWheel)
        car = np.dot(Rot1, car)

        rltWheel += np.array([[-C.RTR], [C.WD / 2]])
        rrtWheel += np.array([[-C.RTR], [-C.WD / 2]])

        rltWheel = np.dot(Rot3, rltWheel)
        rrtWheel = np.dot(Rot3, rrtWheel)
        trail = np.dot(Rot3, trail)

        frWheel += np.array([[x], [y]])
        flWheel += np.array([[x], [y]])
        rrWheel += np.array([[x], [y]])
        rlWheel += np.array([[x], [y]])
        rrtWheel += np.array([[x], [y]])
        rltWheel += np.array([[x], [y]])
        car += np.array([[x], [y]])
        trail += np.array([[x], [y]])

        plt.plot(car[0, :], car[1, :], color)
        plt.plot(trail[0, :], trail[1, :], color)
        plt.plot(frWheel[0, :], frWheel[1, :], color)
        plt.plot(rrWheel[0, :], rrWheel[1, :], color)
        plt.plot(flWheel[0, :], flWheel[1, :], color)
        plt.plot(rlWheel[0, :], rlWheel[1, :], color)
        plt.plot(rrtWheel[0, :], rrtWheel[1, :], color)
        plt.plot(rltWheel[0, :], rltWheel[1, :], color)
        Arrow(x, y, yaw, C.WB * 0.8, color)
        return