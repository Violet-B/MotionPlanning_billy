import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import sys
import os


## Map Configuration
XScale = 2
YScale = 2
current_directory = os.path.dirname(os.path.realpath(__file__))


def read_csv_file(file_path):
    coordinates_list = []
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for row in csv_reader:
            x, y = map(float, row[0:2])
            coordinates_list.append((x * XScale, y * YScale))
    return coordinates_list


def design_rectangles(points, x_offset, y_offset):
    connected_points = []
    for (x, y) in points:
        connected_points.append((x - x_offset, y + y_offset))
        connected_points.append((x - x_offset, y - y_offset))
        connected_points.append((x + x_offset, y - y_offset))
        connected_points.append((x + x_offset, y + y_offset))
        connected_points.append((x - x_offset, y + y_offset))
    return connected_points


def draw_map(obstacle_points, truck_points, parkingspot_points):
    # draw outlines
    x_outlines = [-35, 35, 35, -35, -35]
    y_outlines = [-55, -55, 55, 55, -55]
    x_outlines_scaled = list(map(lambda x : x * XScale, x_outlines))
    y_outlines_scaled = list(map(lambda y : y * YScale, y_outlines))
    plt.plot(x_outlines_scaled, y_outlines_scaled, color='black', linewidth=2)

    # draw obstacles, trucks, and parking spots
    for points, color in zip([obstacle_points, truck_points, parkingspot_points], ['blue', 'red', 'yellow']):
        for i in range(0, len(points), 5):
            x_values, y_values = zip(*points[i:i+5])
            plt.plot(x_values, y_values, color=color, linestyle='-')


def main():
    obstacle_file_path = os.path.join(current_directory, 'csvfiles', 'Obstaclecsv', 'obstacle_coordinates.csv')
    truck_file_path = os.path.join(current_directory, 'csvfiles', 'Truckcsv', 'truck_coordinates.csv')
    parkingspot_path = os.path.join(current_directory, 'csvfiles', 'Goalcsv', 'goal_coordinates.csv')
    ObstacleCoordinates = read_csv_file(obstacle_file_path)
    TruckCoordinates = read_csv_file(truck_file_path)
    ParkingSpotCoordinates = read_csv_file(parkingspot_path)

    obstacle_points = design_rectangles(ObstacleCoordinates, 1 * XScale, 2 * YScale)
    truck_points = design_rectangles(TruckCoordinates, 2 * XScale, 1 * YScale)
    parkingspot_points = design_rectangles(ParkingSpotCoordinates, 2 * XScale, 1 * YScale)

    draw_map(obstacle_points, truck_points, parkingspot_points)

    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()