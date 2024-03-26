import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from collections import OrderedDict
import csv
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PPOPlanner")
import CarConfig.pid_config as pc

# 过于靠近停车位则不再采样
def no_sampling_zone(x, y, angle, goalx, goaly, leftx, rightx, upy, downy):
    index = next((i for i, (x_val, y_val) in enumerate(zip(x, y)) if goalx - leftx < x_val < goalx + rightx and goaly -downy < y_val < goaly + upy), None)

    if index is not None:
        x = x[:index + 1]
        y = y[:index + 1]
        angle = angle[:index + 1]
    return x, y, angle
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PPOPlanner")
import CarConfig.pid_config as pc

# 过于靠近停车位则不再采样
def no_sampling_zone(x, y, angle, goalx, goaly, leftx, rightx, upy, downy):
    index = next((i for i, (x_val, y_val) in enumerate(zip(x, y)) if goalx - leftx < x_val < goalx + rightx and goaly -downy < y_val < goaly + upy), None)

    if index is not None:
        x = x[:index + 1]
        y = y[:index + 1]
        angle = angle[:index + 1]
    return x, y, angle

# 根据三点坐标计算(x2,y2)点处的夹角
def count_angle(x1,y1,x2,y2,x3,y3):
    if x2-x1!=0:
        k1 = (y2-y1)/(x2-x1)
    elif x2-x1==0: 
        k1 = 200000000
    
    if x3-x2!=0:
        k2 = (y2-y3)/(x2-x3)
    elif x3-x2==0:
        k2=200000000
    
    if k1*k2==-1:
        theta = 90
        return theta
    elif k1*k2!=-1:
        theta = math.degrees(math.atan(abs(k2-k1)/(1+k1*k2)))
        return theta

#拐点判断
def is_turning_point(x1,y1,x2,y2,x3,y3):
    distance_threshold = 0.001
    angle_difference_threshold = 5
def is_turning_point(x1,y1,x2,y2,x3,y3):
    distance_threshold = 0.001
    angle_difference_threshold = 5
    dist1 = np.sqrt((x2-x1)**2+(y2-y1)**2)
    dist2 = np.sqrt((x2-x3)**2+(y2-y3)**2)
    d = abs(dist1-dist2)
    Angle = count_angle(x1,y1,x2,y2,x3,y3)
    Angle = count_angle(x1,y1,x2,y2,x3,y3)


    if d>distance_threshold or Angle>angle_difference_threshold:
         return True
    else:
         return False
    
def add_turning_point(x, y, angle, x_turning, y_turning, angle_turning, l, n):
    #将拐点坐标加入list
    for i in range(0,l,n):
        if is_turning_point(x[(i-n)%l],y[(i-n)%l],x[i],y[i],x[(i+n)%l],y[(i+n)%l]):
            x_turning.append(x[i])
            y_turning.append(y[i])
            angle_turning.append(angle[i])
            #判断直拐点处是否被跳过，若有则找到直拐点并加入list
            if not is_turning_point(x[(i-n-n)%l],y[(i-n-n)%l],x[(i-n)%l],y[(i-n)%l],x[(i)%l],y[(i)%l]) and is_turning_point(x[(i)%l],y[(i)%l],x[(i+n)%l],y[(i+n)%l],x[(i+n+n)%l],y[(i+n+n)%l]) and not is_turning_point(x[(i+n)%l],y[(i+n)%l],x[(i+2*n)%l],y[(i+2*n)%l],x[(i+3*n)%l],y[(i+3*n)%l]):
                for k in range(0,n-1,1):
                    if is_turning_point(x[(i+k)%l],y[(i+k)%l],x[(i+k+1)%l],y[(i+k+1)%l],x[(i+k+2)%l],y[(i+k+2)%l]):
                        x_turning.append(x[(i+k+1)%l])
                        y_turning.append(y[(i+k+1)%l])
                        angle_turning.append(angle[(i+k+1)%l])
    
def add_turning_point(x, y, angle, x_turning, y_turning, angle_turning, l, n):
    #将拐点坐标加入list
    for i in range(0,l,n):
        if is_turning_point(x[(i-n)%l],y[(i-n)%l],x[i],y[i],x[(i+n)%l],y[(i+n)%l]):
            x_turning.append(x[i])
            y_turning.append(y[i])
            angle_turning.append(angle[i])
            #判断直拐点处是否被跳过，若有则找到直拐点并加入list
            if not is_turning_point(x[(i-n-n)%l],y[(i-n-n)%l],x[(i-n)%l],y[(i-n)%l],x[(i)%l],y[(i)%l]) and is_turning_point(x[(i)%l],y[(i)%l],x[(i+n)%l],y[(i+n)%l],x[(i+n+n)%l],y[(i+n+n)%l]) and not is_turning_point(x[(i+n)%l],y[(i+n)%l],x[(i+2*n)%l],y[(i+2*n)%l],x[(i+3*n)%l],y[(i+3*n)%l]):
                for k in range(0,n-1,1):
                    if is_turning_point(x[(i+k)%l],y[(i+k)%l],x[(i+k+1)%l],y[(i+k+1)%l],x[(i+k+2)%l],y[(i+k+2)%l]):
                        x_turning.append(x[(i+k+1)%l])
                        y_turning.append(y[(i+k+1)%l])
                        angle_turning.append(angle[(i+k+1)%l])

# 倒数三个点的角度优化
# 对最后不采样的部分人工画轨迹，用切线的方式平滑之
def get_last_three(x_turning, y_turning, angle_turning, goalx, goaly):
    tan_value = (y_turning[-1] - goaly) / (x_turning[-1] - 1.0 - goalx)
    angle_rad = math.atan(tan_value)
    angle_deg = math.degrees(angle_rad)
    a = goalx + 1
    b = goaly
    c = x_turning[-1]
    d = y_turning[-1]
    r = math.sqrt((a - c) ** 2 + (b - d) ** 2) / (2 * math.sin(angle_rad))

    angle_turning[-1] = 180 + 2.0 * angle_deg

    x_turning.append(a - r * math.sin(angle_rad))
    y_turning.append(b - r + r * math.cos(angle_rad))
    angle_turning.append(180.00000 + 1.0 * angle_deg)

    x_turning.append(goalx + 1.0)
    y_turning.append(goaly)
    angle_turning.append(180.00000)

def get_sample_points(x, y, angle, goalx, goaly):
    # 选出非采样区
    x, y, angle = no_sampling_zone(x, y, angle, goalx, goaly, 3, 2, 3, 3)
    angle = list(angle)
    n = pc.C.sample_rate #采样步长
    l=len(x)
    point_number = 0

    x_turning = [x[0]]
    y_turning = [y[0]]
    # 有时初始航向角会有bug（初始为-180左右，然后在第二个点处跳变回真实值），故按照原caragent中的初始航向角设置排除该bug
    if 60 <= angle[0] <= 90 or -270 <= angle[0] <= -240:
        angle_turning = [angle[0]]
    else:
        angle[0] = angle[1]
        angle_turning = [angle[0]]
    
    # 选出拐点（过近则不取）
    add_turning_point(x, y, angle, x_turning, y_turning, angle_turning, l, n)

    #加入最后一个点
    if np.sqrt((x[l-1]-x[l-2])**2+(y[l-1]-y[l-2])**2) > 0.5:
        x_turning.append(x[l-1])
        y_turning.append(y[l-1])
        angle_turning.append(angle[len(angle)-1])

    #消除重复选点
    points = list(OrderedDict.fromkeys(list(zip(x_turning,y_turning,angle_turning))))
    point_number = len(points)
    x_turning,y_turning,angle_turning = zip(*points)
    x_turning = list(x_turning)
    y_turning = list(y_turning)
    angle_turning = list(angle_turning)

    # 最后三个点的建立
    get_last_three(x_turning, y_turning, angle_turning, goalx, goaly)
    point_number += 2

    return x_turning, y_turning, angle_turning, point_number

def show():
    #file path
    data = pd.read_csv('MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Pathcsv/path_coordinates.csv')
    parking_spot = pd.read_csv('MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Goalcsv/goal_coordinates.csv')

    #change the name of 'x' 'y' to the column's name in csv file
    x=data['X']
    y=data['Z']
    angle=data['angle']
    parking_spot_x = parking_spot['X']
    goalx = parking_spot_x[0]
    parking_spot_y = parking_spot['Z']
    goaly = parking_spot_y[0]

    x_turning, y_turning, angle_turning, point_number = get_sample_points(x, y, angle, goalx, goaly)
    #消除重复选点
    points = list(OrderedDict.fromkeys(list(zip(x_turning,y_turning,angle_turning))))
    point_number = len(points)
    x_turning,y_turning,angle_turning = zip(*points)
    x_turning = list(x_turning)
    y_turning = list(y_turning)
    angle_turning = list(angle_turning)

    # 最后三个点的建立
    get_last_three(x_turning, y_turning, angle_turning, goalx, goaly)
    point_number += 2

    return x_turning, y_turning, angle_turning, point_number

def show():
    #file path
    data = pd.read_csv('MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Pathcsv/path_coordinates.csv')
    parking_spot = pd.read_csv('MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Goalcsv/goal_coordinates.csv')

    #change the name of 'x' 'y' to the column's name in csv file
    x=data['X']
    y=data['Z']
    angle=data['angle']
    parking_spot_x = parking_spot['X']
    goalx = parking_spot_x[0]
    parking_spot_y = parking_spot['Z']
    goaly = parking_spot_y[0]

    x_turning, y_turning, angle_turning, point_number = get_sample_points(x, y, angle, goalx, goaly)

    plt.plot(x_turning,y_turning,color='red')   #显示线图
    plt.scatter(x_turning,y_turning,color='blue')    #显示拐点
    plt.axis('equal')
    plt.plot(x_turning,y_turning,color='red')   #显示线图
    plt.scatter(x_turning,y_turning,color='blue')    #显示拐点
    plt.axis('equal')

    # 显示拐点航向角方向
    yaw_list = [np.deg2rad(angle) for angle in angle_turning]
    for i in range(len(x_turning)):
        plt.arrow(x_turning[i], y_turning[i], 0.1 * np.cos(yaw_list[i]), 0.1 * np.sin(yaw_list[i]), head_width=0.1, head_length=1, length_includes_head=False, fc='black', ec='black')
    # 显示拐点航向角方向
    yaw_list = [np.deg2rad(angle) for angle in angle_turning]
    for i in range(len(x_turning)):
        plt.arrow(x_turning[i], y_turning[i], 0.1 * np.cos(yaw_list[i]), 0.1 * np.sin(yaw_list[i]), head_width=0.1, head_length=1, length_includes_head=False, fc='black', ec='black')

    # print(data)
    print('number of points:',point_number)
    print((x_turning, y_turning, angle_turning))

    plt.show()
    # print(data)
    print('number of points:',point_number)
    print((x_turning, y_turning, angle_turning))

    plt.show()

    csv_file_path = 'MotionPlanning_billy/outputcsvfiles/opimaized_path.csv'
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['x_turning', 'y_turning', 'angle_turning'])
        for x, y, angle in zip(x_turning, y_turning, angle_turning):
            csv_writer.writerow([x, y, angle])
    print(f'Data has been written to {csv_file_path}')

def produce_files(index):
    for i in range(1, index + 1):
        data = pd.read_csv(f'MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Pathcsv/path_coordinates_{i}.csv')
        parking_spot = pd.read_csv(f'MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Goalcsv/goal_coordinates_{i}.csv')

        #change the name of 'x' 'y' to the column's name in csv file
        x=data['X']
        y=data['Z']
        angle=data['angle']
        parking_spot_x = parking_spot['X']
        goalx = parking_spot_x[0]
        parking_spot_y = parking_spot['Z']
        goaly = parking_spot_y[0]

        x_turning, y_turning, angle_turning, point_number = get_sample_points(x, y, angle, goalx, goaly)

        csv_file_path = f'MotionPlanning_billy/outputcsvfiles/opimaized_path_{i}.csv'
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['x_turning', 'y_turning', 'angle_turning'])
            for x, y, angle in zip(x_turning, y_turning, angle_turning):
                csv_writer.writerow([x, y, angle])
        print(f'Data has been written to {csv_file_path}')

if __name__ == '__main__':
    # show()
    produce_files(3)