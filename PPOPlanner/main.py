from CarConfig.pid_config import C
from MapData.MapGen import plot_map, get_rl_path
from TrajectoryTrack.path_smooth import generate_path
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Init map infomation
    obs_points, truck_points, park_points = plot_map(show_flag=False)
    rl_states = get_rl_path()
    states_downsample = rl_states[::C.sample_rate]

    # Paht Smoothing
    x, y, yaw, direct, path_x, path_y = generate_path(states_downsample)


    # animation
    plt.cla()
    plt.plot(path_x, path_y, color='gray', linewidth=2)
    plt.show()