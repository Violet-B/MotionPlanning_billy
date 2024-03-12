class C:
    # PID config
    Kp = 0.3  # proportional gain

    # system config
    # Ld = 2.6  # look ahead distance
    # kf = 0.1  # look forward gain
    # dt = 0.1  # T step
    # dist_stop = 0.7  # stop distance
    # dc = 0.0

    Ld = 2.6  # look ahead distance
    kf = 0.02  # look forward gain
    dt = 0.05  # T step
    dist_stop = 0.7  # stop distance
    dc = 0.0

    maxTime = 100
    sample_rate = 8

    # # vehicle config
    # RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    # RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    # W = 2.4  # [m] width of vehicle
    # WD = 0.7 * W  # [m] distance between left-right wheels
    # WB = 2.5  # [m] Wheel base
    # TR = 0.44  # [m] Tyre radius
    # TW = 0.7  # [m] Tyre width
    # MAX_STEER = 0.30
    # MAX_ACCELERATION = 5.0

    RF = 3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.5  # [m] distance from rear to vehicle back end of vehicle
    W = 2  # [m] width of vehicle
    WD = 1 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width
    MAX_STEER = 1.20 #max steering angle (rad)
    MAX_ACCELERATION = 10.0
