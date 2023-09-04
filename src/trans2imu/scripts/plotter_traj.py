#!/usr/bin/env python
#coding=utf-8


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取轨迹数据文件
file_path = "odom.txt"
# file_path = "trajectory1.txt"

east_coords = []
north_coords = []
up_coords = []

with open(file_path, "r") as file:
    lines = file.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            east_coords.append(float(values[0])-(-2159255.5944))
            north_coords.append(float(values[1])-(4396935.8873))
            up_coords.append(float(values[2])-(4071171.0834))

# #绘制2D轨迹
# plt.figure()
# plt.plot(east_coords, north_coords, label='trajectory', linewidth=2)
# plt.xlabel('East (m)')
# plt.ylabel('North (m)')
# plt.title('2D Trajectory in NEU Coordinates')
# plt.legend()
# plt.axis('equal')
# plt.grid(True)
# plt.show()



# 绘制3D轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(east_coords, north_coords, up_coords, label='trajectory', linewidth=2)
ax.set_xlabel('East (m)')
ax.set_ylabel('North (m)')
ax.set_zlabel('Up (m)')
ax.set_title('3D Trajectory in NEU Coordinates')
ax.legend()
# plt.axis('equal')
plt.show()
