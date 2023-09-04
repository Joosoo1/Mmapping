#!/usr/bin/env python
import rospy
import rosbag
from rslidar_msgs.msg import rslidarScan
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

# 导入 message_filters 库来实现消息的同步订阅
import message_filters

# 初始化ROS节点
rospy.init_node('multi_topic_dataset_builder')

# 创建一个ROS bag文件
bag = rosbag.Bag('robot1.bag', 'w')  # 'w' 表示写入模式，您可以替换 'dataset.bag' 为您的文件名

# 定义回调函数，处理接收到的消息
def callback(imu_msg, odom_msg):
    imu_msg.orientation.x = odom_msg.pose.pose.orientation.x
    imu_msg.orientation.y = odom_msg.pose.pose.orientation.y
    imu_msg.orientation.z = odom_msg.pose.pose.orientation.z
    imu_msg.orientation.w = odom_msg.pose.pose.orientation.w

    # 将处理后的消息添加到ROS bag
    bag.write('/robot1/imu', imu_msg) 
    bag.write('/robot1/odom', odom_msg)
    # bag.write('/robot1/rslidar_packets', scan_msg)
    # bag.write('/robot1//points_raw', pointcloud_msg)


# 订阅图像主题
imu_sub = message_filters.Subscriber('/fixposition/corrimu', Imu)  
odom_sub = message_filters.Subscriber('/fixposition/odometry', Odometry)  
scan_sub = message_filters.Subscriber('/rslidar_packets', rslidarScan)  
pointcloud_sub = message_filters.Subscriber('/points_raw', PointCloud2)


# 创建消息过滤器，将两个消息进行同步订阅
ts = message_filters.ApproximateTimeSynchronizer([imu_sub, odom_sub], 100, 0.1)
ts.registerCallback(callback)

# 开始ROS节点
rospy.spin()

# 关闭ROS bag文件
bag.close()
