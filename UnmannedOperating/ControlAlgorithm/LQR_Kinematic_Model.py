import PlanningAlgorithm.reeds_shepp as rs
import numpy as np
import math
import matplotlib.pyplot as plt
from enum import Enum
from config import ParaConfig
from ControlAlgorithm.utils import *


def control_Kinematic(path_list, all_path, Tcar):
    # Generate Path
    x_ref, y_ref, yaw_ref, direct, curv = [], [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []

    for p in path_list:
        x_ref.append(p.x)
        y_ref.append(p.y)
        yaw_ref.append(p.yaw)
        direct.append(p.direction)
        
        irc, _ = rs.calc_curvature(p.x, p.y, p.yaw, p.direction)
        curv.append(irc)

    # Init Controller
    maxTime = 100.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = \
    x_ref[0][0], y_ref[0][0], yaw_ref[0][0], direct[0][0]
    lat_controller = LatController()
    lon_controller = LonController()

    # Draw Path
    for x, y, yaw, gear, k in zip(x_ref, y_ref, yaw_ref, direct, curv):
        t = 0.0

        if gear[0] == 1.0:
            direct = Gear.GEAR_DRIVE
        else:
            direct = Gear.GEAR_REVERSE

        ref_trajectory = TrajectoryAnalyzer(x, y, yaw, k)

        vehicle_state = VehicleState(x=x0, y=y0, yaw=yaw0, v=0.1, gear=direct)

        while t < maxTime:

            dist = math.hypot(vehicle_state.x - x[-1], vehicle_state.y - y[-1])

            if gear[0] > 0:
                target_speed = 25.0 / 3.6
            else:
                target_speed = 15.0 / 3.6

            delta_opt, theta_e, e_cg = \
                lat_controller.ComputeControlCommand(vehicle_state, ref_trajectory)

            a_opt = lon_controller.ComputeControlCommand(target_speed, vehicle_state, dist)

            vehicle_state.UpdateVehicleState(delta_opt, a_opt, e_cg, theta_e, direct)

            t += ts

            if dist <= 0.5:
                break

            x_rec.append(vehicle_state.x)
            y_rec.append(vehicle_state.y)
            yaw_rec.append(vehicle_state.yaw)

            dy = (vehicle_state.yaw - yaw_old) / (vehicle_state.v * ts)
            # steer = rs.pi_2_pi(-math.atan(wheelbase_ * dy))

            yaw_old = vehicle_state.yaw
            x0 = x_rec[-1]
            y0 = y_rec[-1]
            yaw0 = yaw_rec[-1]

            plt.cla()
            plt.plot(x_rec, y_rec, linewidth=2.0, color='gray')
            # plt.plot(x[ind], y[ind], '.r')
            Tcar.draw_car(x0, y0, yaw0, -vehicle_state.steer)
            plt.axis("equal")
            plt.title("LQR (Kinematic): v=" + str(vehicle_state.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    control_Kinematic()