import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import sys
import os

# Map Configuration
XScale = float(2)
YScale = float(2)
plt.figure(figsize=(7, 11))
current_directory = os.path.dirname(os.path.realpath(__file__))


def read_pos_file(file_path):
    coordinates_list = []
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for row in csv_reader:
            x, y = map(float, row[0:2])
            coordinates_list.append([x * XScale, y * YScale])
    return coordinates_list

def read_path_file(file_path):
    coordinates_list = []
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for row in csv_reader:
            x, y, yaw = map(float, row[0:3])
            coordinates_list.append([x * XScale, y * YScale, yaw])
    return coordinates_list


def design_rectangles(points, x_offset, y_offset):
    connected_points = []
    for (x, y) in points:
        connected_points.extend([(x - x_offset, y + y_offset),
                                 (x - x_offset, y - y_offset),
                                 (x + x_offset, y - y_offset),
                                 (x + x_offset, y + y_offset),
                                 (x - x_offset, y + y_offset)])
    return connected_points


def draw_map(obstacle_points, truck_points, parkingspot_points):
    # draw outlines
    x_outlines = [-35, 35, 35, -35, -35]
    y_outlines = [-55, -55, 55, 55, -55]
    x_outlines_scaled = [x * XScale for x in x_outlines]
    y_outlines_scaled = [y * YScale for y in y_outlines]
    plt.plot(x_outlines_scaled, y_outlines_scaled, color='black', linewidth=2)

    # draw obstacles, trucks, and parking spots
    for points, color in zip([obstacle_points, truck_points, parkingspot_points], ['blue', 'red', 'yellow']):
        for i in range(0, len(points), 5):
            x_values, y_values = zip(*points[i:i+5])
            plt.plot(x_values, y_values, color=color, linestyle='-')


def plot_map(show_flag=False):
    obstacle_file_path = os.path.join(current_directory, 'csvfiles', 'Obstaclecsv', 'obstacle_coordinates.csv')
    truck_file_path = os.path.join(current_directory, 'csvfiles', 'Truckcsv', 'truck_coordinates.csv')
    parkingspot_path = os.path.join(current_directory, 'csvfiles', 'Goalcsv', 'goal_coordinates.csv')
    obstacle_points = design_rectangles(read_pos_file(obstacle_file_path), 1 * XScale, 2 * YScale)
    truck_points = design_rectangles(read_pos_file(truck_file_path), 2 * XScale, 1 * YScale)
    parkingspot_points = design_rectangles(read_pos_file(parkingspot_path), 2 * XScale, 1 * YScale)
    if show_flag:
        draw_map(obstacle_points, truck_points, parkingspot_points)
        plt.axis("equal")
        plt.show()
        print("map generate successfully")
    return obstacle_points, truck_points, parkingspot_points


def get_rl_path():
    rl_file_path = os.path.join(current_directory, 'csvfiles', 'Pathcsv', 'path_coordinates.csv')
    rl_points = read_path_file(rl_file_path)
    return rl_points

if __name__ == '__main__':
    plot_map()