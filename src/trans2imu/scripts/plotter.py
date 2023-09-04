#!/usr/bin/env python
#coding=utf-8
import matplotlib.pyplot as plt

# 全局变量用于存储接收到的数据
x_data = []
y_data = []
z_data = []
t_data = []

# 从文本文件中读取加速度数据
with open('data.txt', 'r') as file:
    lines = file.readlines()
    for line in lines:
        values = line.strip().split()  # 假设数据以空格分隔
        if len(values) == 4:  # 假设每行数据有4个值：时间、X加速度、Y加速度、Z加速度
            t = float(values[0])  # 获取时间
            x_acc = float(values[1])  # 获取X加速度
            y_acc = float(values[2])  # 获取Y加速度
            z_acc = float(values[3])  # 获取Z加速度

            # 添加数据到列表
            t_data.append(t)
            x_data.append(x_acc)
            y_data.append(y_acc)
            z_data.append(z_acc)

# 创建一个新的图形窗口
plt.figure()

# 配置支持中文显示
plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei']  # 使用文泉驿正黑字体，可以根据需要选择其他字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


# 绘制三个加速度数据随时间变化的曲线图
plt.plot(t_data, x_data, 'r', label='X-Acceleration')
plt.plot(t_data, y_data, 'b', label='Y-Acceleration')
plt.plot(t_data, z_data, 'g', label='Z-Acceleration')
plt.xlabel('时间 (秒)')
plt.ylabel('加速度')
plt.title('加速度随时间变化')
plt.legend()
plt.grid(True)

plt.show()


