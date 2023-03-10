from enviroment import Obstacle
import matplotlib.pyplot as plt
from TrailerCar import TrailerCar
from config import ParaConfig
import planner as P
import time
import math
import algorithm.reeds_shepp as rs
import pickle
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../MotionPlanning/")

def draw_path(Tcar, path, C, Obs):
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction

    obsx, obsy = Obs.get_obs_shape()
    barrierx, barriery = Obs.design_obstacles()

    plt.pause(10)

    for k in range(len(x)):
        plt.cla()
        plt.plot(obsx, obsy, 'sr')
        plt.plot(barrierx, barriery, 'sk')
        plt.plot(x, y, linewidth=1.5, color='r')

        if k < len(x) - 2:
            dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
            steer = rs.pi_2_pi(math.atan(C.WB * dy / direction[k]))
        else:
            steer = 0.0

        Tcar.draw_model(x[k], y[k], yaw[k], steer)
        plt.pause(0.0001)


def main():
    # Init Para
    C = ParaConfig()

    # Init Obstacle
    Obs = Obstacle()
    boundaryx, boundaryy = Obs.get_boundry_shape()
    barrierx, barriery = Obs.design_obstacles()

    # Init Postions
    startx, starty, syawh = Obs.get_start_pos()
    loadx, loady, lyawh = Obs.get_load_pos()
    unloadx, unloady, uyawh = Obs.get_unload_pos()

    # Init Car
    Tcar = TrailerCar()

    # Path Planning
    print("start!")
    oox, ooy = (barrierx+boundaryx)[:], (barriery+boundaryy)[:]
    t0 = time.time()

    if C.Train:
        path_0 = P.hybrid_astar_planning(
            startx, starty, syawh, 
            loadx, loady, lyawh, 
            oox, ooy, C.XY_RESO, C.YAW_RESO)

        path_1 = P.hybrid_astar_planning( 
            loadx, loady, lyawh, 
            unloadx, unloady, uyawh,
            oox, ooy, C.XY_RESO, C.YAW_RESO)

        # path_2 = P.hybrid_astar_planning(
        #     unloadx, unloady, uyawh,
        #     startx, starty, syawh, 
        #     oox, ooy, C.XY_RESO, C.YAW_RESO)
        
        path_0.union([path_1])
        path = path_0

        output_hal = open("UnmannedOperating/data/trailer_path_" + str(C.INDEX) + ".pkl", 'wb')
        str_path = pickle.dumps(path)
        output_hal.write(str_path)
        output_hal.close()
        t1 = time.time()
        print("running T: ", t1 - t0)
    else:
        with open("UnmannedOperating/data/trailer_path_" + str(C.INDEX) + ".pkl",'rb') as file:
                path = pickle.loads(file.read())

    # init Draw
    plt.axis("equal")
    plt.xlim(0, 100)
    plt.ylim(0, 100)

    # test draw car
    # Tcar.draw_model(startx, starty, syawh, 0.0)
    # Tcar.draw_model(loadx, loady, lyawh, 0.0)
    # Tcar.draw_model(unloadx, unloady, uyawh, 0.0)
    
    # Draw Path
    draw_path(Tcar, path, C, Obs)

    # Done
    plt.show()
    print("Done")

if __name__ == '__main__':
    main()