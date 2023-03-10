import numpy as np

class ParaConfig:  # Parameter config
    PI = np.pi

    Train = True
    INDEX = 2

    XY_RESO = 2.0  # [m]
    YAW_RESO = np.deg2rad(15.0)  # [rad]
    MOVE_STEP = 0.4  # [m] path interporate resolution
    N_STEER = 20.0  # steer command number
    COLLISION_CHECK_STEP = 5  # skip number for collision check
    EXTEND_BOUND = 1  # collision check range extended

    GEAR_COST = 100.0  # switch back penalty cost
    BACKWARD_COST = 5.0  # backward penalty cost
    STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
    STEER_ANGLE_COST = 1.0  # steer angle penalty cost
    H_COST = 15.0  # Heuristic cost penalty cost

    RF = 6.5  # [m] distance from rear to vehicle front end of vehicle
    RB = 2.0  # [m] distance from rear to vehicle back end of vehicle
    W = 3.0  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 3.5  # [m] Wheel base
    TR = 0.5  # [m] Tyre radius
    TW = 1  # [m] Tyre width
    MAX_STEER = 0.6  # [rad] maximum steering angle


    # Controller Config
    ts = 0.1  # [s]
    l_f = 1.165  # [m]
    l_r = 1.165  # [m]
    max_iteration = 150
    eps = 0.01

    matrix_q = [0.5, 0.0, 1.0, 0.0]
    matrix_r = [1.0]

    state_size = 4

    max_acceleration = 5.0  # [m / s^2]
    max_steer_angle = np.deg2rad(30)  # [rad]
    max_speed = 35 / 3.6  # [m / s]