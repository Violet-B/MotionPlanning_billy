import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from collections import OrderedDict
import csv
import os

current_directory = os.path.dirname(os.path.realpath(__file__))

#file path
# data = pd.read_csv('AutoPark_Lib_sim2real_v1.0/Assets/Examples/MAPD/csvfiles/Pathcsv/path_coordinates.csv')
data = pd.read_csv('MotionPlanning_billy/PPOPlanner/MapData/csvfiles/Pathcsv/path_coordinates.csv')
print(data)

x=data['X']
y=data['Z']
angle=data['angle']

sampling_rate = 1
distance_threshold = 0.01
angle_difference_threshold = 5
n = 10 #sampling stepsize

l=len(x)
point_number = 0
x_downsampled = x[::sampling_rate]
y_downsampled = y[::sampling_rate]

# 根据三点坐标计算(x2,y2)点处的夹角
def calc_angle(x1,y1,x2,y2,x3,y3):
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
def IsTurningPoint(x1,y1,x2,y2,x3,y3):
    dist1 = np.sqrt((x2-x1)**2+(y2-y1)**2)
    dist2 = np.sqrt((x2-x3)**2+(y2-y3)**2)
    d = abs(dist1-dist2)
    angle = calc_angle(x1,y1,x2,y2,x3,y3)


    if d>distance_threshold or angle>angle_difference_threshold:
         return True
    else:
         return False

x_turning = [x[0]]
y_turning = [y[0]]
angle_turning = [angle[0]]

#将拐点坐标加入list
for i in range(0,l,n):
    if IsTurningPoint(x[(i-n)%l],y[(i-n)%l],x[i],y[i],x[(i+n)%l],y[(i+n)%l]):
        x_turning.append(x[i])
        y_turning.append(y[i])
        angle_turning.append(angle[i])
        #判断直拐点处是否被跳过，若有则找到直拐点并加入list
        if (not IsTurningPoint(x[(i-n-n)%l], y[(i-n-n)%l], x[(i-n)%l], y[(i-n)%l], x[(i)%l], y[(i)%l]) and 
            IsTurningPoint(x[(i)%l], y[(i)%l], x[(i+n)%l], y[(i+n)%l], x[(i+n+n)%l], y[(i+n+n)%l]) and 
            not IsTurningPoint(x[(i+n)%l], y[(i+n)%l], x[(i+2*n)%l], y[(i+2*n)%l], x[(i+3*n)%l], y[(i+3*n)%l])):
            for k in range(0, n-1, 1):
                if IsTurningPoint(x[(i+k)%l], y[(i+k)%l], x[(i+k+1)%l], y[(i+k+1)%l], x[(i+k+2)%l], y[(i+k+2)%l]):
                    x_turning.append(x[(i+k+1)%l])
                    y_turning.append(y[(i+k+1)%l])
                    angle_turning.append(angle[(i+k+1)%l])


#消除重复选点
points = list(OrderedDict.fromkeys(list(zip(x_turning,y_turning,angle_turning))))
point_number = len(points)
x_turning,y_turning,angle_turning = zip(*points)
x_turning = list(x_turning)
y_turning = list(y_turning)
angle_turning = list(angle_turning)
# angle_turning[len(angle_turning)-1] = -180

x1 = min(x_turning)
y1 = min(y_turning)

plt.plot(x_turning,y_turning,color='red')   #显示线图
plt.axis('equal')
plt.scatter(x_turning,y_turning,color='blue')    #显示拐点

# plt.plot(x_downsampled,y_downsampled,color='purple')

# plt.scatter(x_downsampled,y_downsampled,color='green')

print('number of points:',point_number)
plt.show()

# print((x_turning,y_turning))

# csv_file_path = 'MotionPlanning_ownversion/outputcsvfiles/opimaized_path.csv'
csv_file_path = 'MotionPlanning_billy/outputcsvfiles/opimaized_path.csv'

with open(csv_file_path, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['x_turning', 'y_turning', 'angle_turning'])
    for x, y, angle in zip(x_turning, y_turning, angle_turning):
        csv_writer.writerow([x, y, angle])

print(f'Data has been written to {csv_file_path}')