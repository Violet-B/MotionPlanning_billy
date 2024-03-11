from CarConfig.pid_config import C
from MapData.MapGen import plot_map, get_rl_path, get_rl_goal
from TrajectoryTrack.path_smooth import generate_path
from Sim2Real.path_optimize import get_sample_points
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Init map infomation
    obs_points, truck_points, park_points = plot_map(show_flag=False)
    rl_states = get_rl_path()
    x, y, angle = zip(*rl_states)
    goalx, goaly = get_rl_goal()
    x_turning, y_turning, angle_turning, point_number = get_sample_points(x, y, angle, goalx, goaly)
    states_downsample = [[a, b, c] for a, b, c in zip(x_turning, y_turning, angle_turning)]
    # states_downsample = rl_states[::C.sample_rate]
    print(states_downsample)

    # Paht Smoothing
    x, y, yaw, direct, path_x, path_y = generate_path(states_downsample)
    


    # animation
    plt.cla()
    plt.plot(path_x, path_y, color='gray', linewidth=2)
    plt.axis('equal')
    plt.show()