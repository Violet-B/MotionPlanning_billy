import numpy as np

class ParaConfig:  # Parameter config
    PI = np.pi

    XY_RESO = 2.0  # [m]
    YAW_RESO = np.deg2rad(15.0)  # [rad]
    GOAL_YAW_ERROR = np.deg2rad(3.0)  # [rad]
    MOVE_STEP = 0.2  # [m] path interporate resolution
    N_STEER = 20.0  # number of steer command
    COLLISION_CHECK_STEP = 10  # skip number for collision check
    EXTEND_AREA = 5.0  # [m] map extend length

    GEAR_COST = 100.0  # switch back penalty cost
    BACKWARD_COST = 5.0  # backward penalty cost
    STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
    STEER_ANGLE_COST = 1.0  # steer angle penalty cost
    SCISSORS_COST = 200.0  # scissors cost
    H_COST = 10.0  # Heuristic cost

    W = 3.0  # [m] width of vehicle
    WB = 3.5  # [m] wheel base: rear to front steer
    WD = 0.7 * W  # [m] distance between left-right wheels
    RF = 4.5  # [m] distance from rear to vehicle front end of vehicle
    RB = 1.0  # [m] distance from rear to vehicle back end of vehicle

    RTR = 5.0  # [m] rear to trailer wheel
    RTF = 1.0  # [m] distance from rear to vehicle front end of trailer
    RTB = 6.0  # [m] distance from rear to vehicle back end of trailer
    TR = 0.5  # [m] tyre radius
    TW = 1.0  # [m] tyre width
    MAX_STEER = 0.6  # [rad] maximum steering angle