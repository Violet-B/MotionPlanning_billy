from enviroment import Obstacle
import matplotlib.pyplot as plt
from TrailerCar import TrailerCar
from config import ParaConfig
import planner as P

def main():
    # Init Para
    C = ParaConfig()

    # Init Obstacle
    Obs = Obstacle()
    obsx, obsy = Obs.get_obs_shape()
    barrierx, barriery = Obs.design_obstacles()

    # Init Postions
    startx, starty, syawh, syawt = Obs.get_start_pos()
    loadx, loady, lyawh, lyawt = Obs.get_load_pos()
    unloadx, unloady, uyawh, uyawt = Obs.get_unload_pos()

    # Init Car
    Tcar = TrailerCar()

    # Path Planning
    oox, ooy = barrierx[:], barriery[:]
    path = P.hybrid_astar_planning(
        startx, starty, syawh, syawt, 
        loadx, loady, lyawh, lyawt, 
        oox, ooy, C.XY_RESO, C.YAW_RESO)

    # Draw
    print("start!")
    plt.figure(figsize=(100,100))
    plt.axis("equal")
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    Tcar.draw_model(startx, starty, syawh, syawt, 0.0)
    Tcar.draw_model(loadx, loady, lyawh, lyawt, 0.0)
    Tcar.draw_model(unloadx, unloady, uyawh, uyawt, 0.0)
    plt.plot(obsx, obsy, 'sk')
    plt.plot(startx, starty, 'o')
    plt.plot(loadx, loady, 'o')
    plt.plot(unloadx, unloady, 'o')
    plt.show()
    print("Done")

if __name__ == '__main__':
    main()