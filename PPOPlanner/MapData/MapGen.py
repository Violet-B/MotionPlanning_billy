import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import sys
import os

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../MotionPlanning/")
current_directory = os.path.dirname(os.path.realpath(__file__))

def read_csv_file(file_path):
    coordinates_list = []

    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        
        # Skip the header
        next(csv_reader)

        for row in csv_reader:
            # Convert string values to float and create a tuple
            x, y = map(float, row[0:2])
            coordinates_list.append((x, y))

    return coordinates_list

def plot_rectangles(coordinates):
    fig, ax = plt.subplots()

    for (x, y) in coordinates:
        # Create a rectangle centered at (x, y) with width=4 and height=2
        rect = Rectangle((x - 2, y - 1), 4, 2, linewidth=1, edgecolor='black', facecolor='yellow')
        ax.add_patch(rect)

def design_24rectangles(points):
    connected_points = []

    for (x, y) in points:

        # Append the four points
        connected_points.append((x - 1, y + 2))
        connected_points.append((x - 1, y - 2))
        connected_points.append((x + 1, y - 2))
        connected_points.append((x + 1, y + 2))
        connected_points.append((x - 1, y + 2))
    
    # x_values, y_values = zip(*connected_points)

    # return x_values, y_values
    return connected_points

def design_42rectangles(points):
    connected_points = []

    for (x, y) in points:

        # Append the four points
        connected_points.append((x - 2, y + 1))
        connected_points.append((x - 2, y - 1))
        connected_points.append((x + 2, y - 1))
        connected_points.append((x + 2, y + 1))
        connected_points.append((x - 2, y + 1))
    
    # x_values, y_values = zip(*connected_points)

    # return x_values, y_values
    return connected_points

def draw_map(obstacle_points, truck_points, parkingspot_points):
    # draw outlines
    x_values = [-35, 35, 35, -35, -35]
    y_values = [-55, -55, 55, 55, -55]
    plt.plot(x_values, y_values, color='black', linewidth=2)

    # draw obstacles
    for i in range(0, len(obstacle_points), 5):
        x_values, y_values = zip(*obstacle_points[i:i+5])
        plt.plot(x_values, y_values, color='blue', linestyle='-')

    # draw trucks
    for i in range(0, len(truck_points), 5):
        x_values, y_values = zip(*truck_points[i:i+5])
        plt.plot(x_values, y_values, color='red', linestyle='-')
        
    # draw parking spot
    for i in range(0, len(parkingspot_points), 5):
        x_values, y_values = zip(*parkingspot_points[i:i+5])
        plt.plot(x_values, y_values, color='yellow', linestyle='-')

def main():

    obstacle_file_path = os.path.join(current_directory, 'csvfiles', 'Obstaclecsv', 'obstacle_coordinates.csv')
    truck_file_path = os.path.join(current_directory, 'csvfiles', 'Truckcsv', 'truck_coordinates.csv')
    parkingspot_path = os.path.join(current_directory, 'csvfiles', 'Goalcsv', 'goal_coordinates.csv')
    ObstacleCoordinates = read_csv_file(obstacle_file_path)
    TruckCoordinates = read_csv_file(truck_file_path)
    ParkingSpotCoordinates = read_csv_file(parkingspot_path)

    obstacle_points = design_24rectangles(ObstacleCoordinates)
    truck_points = design_42rectangles(TruckCoordinates)
    parkingspot_points = design_42rectangles(ParkingSpotCoordinates)

    draw_map(obstacle_points, truck_points, parkingspot_points)

    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()