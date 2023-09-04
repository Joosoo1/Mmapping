#!/usr/bin/env python
#coding=utf-8

import matplotlib.pyplot as plt

# 从第一个文本文件读取数据
file1 = "trajectory3.txt"
data1_x = []
data1_y = []
data1_z = []

with open(file1, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data1_x.append(float(values[0]))
            data1_y.append(float(values[1]))
            data1_z.append(float(values[2]))

# 从第二个文本文件读取数据
file2 = "robot1_gps.txt"
data2_x = []
data2_y = []
data2_z = []

with open(file2, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data2_x.append(float(values[0]))
            data2_y.append(float(values[1]))
            data2_z.append(float(values[2]))

# 从第三个文本文件读取数据
file3 = "robot2_gps.txt"
data3_x = []
data3_y = []
data3_z = []

with open(file3, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data3_x.append(float(values[0]))
            data3_y.append(float(values[1]))
            data3_z.append(float(values[2]))

# 从第四个文本文件读取数据
file4 = "fix.txt"
data4_x = []
data4_y = []
data4_z = []

with open(file4, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data4_x.append(float(values[0]))
            data4_y.append(float(values[1]))
            data4_z.append(float(values[2]))

# 从第四个文本文件读取数据
file5 = "odom1.txt"
data5_x = []
data5_y = []
data5_z = []

with open(file5, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data5_x.append(float(values[0])+(-3.988941321301442))
            data5_y.append(float(values[1])+(-9.187375316343621))
            data5_z.append(float(values[2])+(-0.8960078788203552))

# 从第四个文本文件读取数据
file6 = "odom2.txt"
data6_x = []
data6_y = []
data6_z = []

with open(file6, "r") as f:
    lines = f.readlines()
    for line in lines:
        values = line.strip().split()
        if len(values) == 3:
            data6_x.append(float(values[0])+(-18.963133886158133))
            data6_y.append(float(values[1])+(137.11981188180818))
            data6_z.append(float(values[2])+(0.5064943195171878))
# 绘制轨迹
plt.figure()
plt.plot(data1_x, data1_y, label="novatel", linestyle="-", marker="o", color="blue")
# plt.plot(data2_x, data2_y, label="gps1", linestyle="-", marker="x", color="red")
# plt.plot(data3_x, data3_y, label="gps2", linestyle="-", marker="*", color="green")
plt.plot(data4_x, data4_y, label="RTK", linestyle="-", marker="^", color="red")
# plt.plot(data5_x, data5_y, label="odom1", linestyle="-", marker="^", color="red")
# plt.plot(data6_x, data6_y, label="odom1", linestyle="-", marker="^", color="green")
# plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Trajectory Comparison")
plt.legend()
plt.grid(True)

# # 设置纵轴和横轴尺度一致
plt.axis('equal')

# 显示图形
plt.show()


# 绘制3D轨迹
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(data1_x, data1_y, data1_z, label="odom", linestyle="-", marker="o", color="blue")
# ax.plot(data2_x, data2_y, data2_z, label="INS", linestyle="-", marker="x", color="red")
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('3D Trajectory Comparison')
# ax.legend()
# # plt.axis('equal')
# plt.show()
