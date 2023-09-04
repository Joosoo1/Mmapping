#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Imu
from novatel_oem7_msgs.msg import INSPVA
from novatel_oem7_msgs.msg import CORRIMU
import numpy as np
import math

# 导入 message_filters 库来实现消息的同步订阅
import message_filters

# 初始化ROS节点
rospy.init_node('multi_topic_dataset_builder')

# 创建一个ROS bag文件
# bag = rosbag.Bag('robot1.bag', 'w')  # 'w' 表示写入模式，您可以替换 'dataset.bag' 为您的文件名

#创建一个imu消息对象
msg = Imu()

# 定义回调函数，处理接收到的消息
def callback(imu_msg, ins_msg):
    # msg.header = imu_msg.header
    # msg.angular_velocity.x = imu_msg.pitch_rate
    # msg.angular_velocity.y = imu_msg.roll_rate
    # msg.angular_velocity.z = imu_msg.yaw_rate
    # msg.linear_acceleration.x = imu_msg.longitudinal_acc


    


    # msg.header = ins_msg.header

    pitch_rate = imu_msg.pitch_rate
    roll_rate = imu_msg.roll_rate
    yaw_rate = imu_msg.yaw_rate
    lateral_acc = imu_msg.lateral_acc
    longitudinal_acc = imu_msg.longitudinal_acc
    vertical_acc = imu_msg.vertical_acc
    roll_matrix = np.array([[1, 0, 0],
                         [0, math.cos(roll_rate), -math.sin(roll_rate)],
                         [0, math.sin(roll_rate), math.cos(roll_rate)]])

    pitch_matrix = np.array([[math.cos(pitch_rate), 0, math.sin(pitch_rate)],
                            [0, 1, 0],
                            [-math.sin(pitch_rate), 0, math.cos(pitch_rate)]])

    yaw_matrix = np.array([[math.cos(yaw_rate), -math.sin(yaw_rate), 0],
                            [math.sin(yaw_rate), math.cos(yaw_rate), 0],
                            [0, 0, 1]])

    # 总旋转矩阵
    rotation_matrix = np.dot(yaw_matrix, np.dot(pitch_matrix, roll_matrix))

    # 加速度数据向量
    acceleration_vector = np.array([[lateral_acc],
                                    [longitudinal_acc],
                                    [vertical_acc]])

    # 将加速度数据从车辆坐标系转换到ENU坐标系
    acceleration_enu = np.dot(rotation_matrix, acceleration_vector)

    # 输出结果
    print("Acceleration in ENU coordinates:")
    print(acceleration_enu)



    

    # 将处理后的消息添加到ROS bag

    # bag.write('/robot1/rslidar_packets', scan_msg)
    # bag.write('/robot1//points_raw', pointcloud_msg)


# 订阅图像主题
imu_sub = message_filters.Subscriber('/novatel/oem7/corrimu', CORRIMU)  
ins_sub = message_filters.Subscriber('/novatel/oem7/inspva', INSPVA)  


# 创建消息过滤器，将两个消息进行同步订阅
ts = message_filters.ApproximateTimeSynchronizer([imu_sub, ins_sub], 100, 0.1)
ts.registerCallback(callback)

# 开始ROS节点
rospy.spin()

# 关闭ROS bag文件
# bag.close()
