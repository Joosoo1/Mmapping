#!/usr/bin/env python
#coding=utf-8

import rospy
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
# import pyproj
import math



# 经度纬度高度转ECEF的X,Y,Z
# 注意输入的经度和纬度的单位是度，高度单位是米；输出的X,Y,Z的单位是米
def lla2xyz(longitude, latitude, h):
    longitude = longitude / 180.0 * math.pi
    latitude = latitude / 180.0 * math.pi

    a = 6378137.0
    b = 6356752.3142
    e2 = (a ** 2 - b ** 2) / a ** 2
    N = a / math.sqrt(1 - e2 * math.sin(latitude) * math.sin(latitude))
    x = (N + h) * math.cos(latitude) * math.cos(longitude)
    y = (N + h) * math.cos(latitude) * math.sin(longitude)
    z = (N * (1 - e2) + h) * math.sin(latitude)
    return [x, y, z]


# 输入2个ECEF坐标，输出一个向量(以第1个点为原点)
# longitude1和latitude1分别表示第1个点的经度和纬度，x1,y1和z1表示第1个点的ECEF坐标
# x2,y2,z2表示第2个点的ECEF坐标
# 返回一个列表，里面有三个元素，表示以第1个点为原点，东北天为xyz轴方向，在此坐标系下，第2个点的坐标
def xyz2enu(longitude1, latitude1, x1, y1, z1, x2, y2, z2):
    longitude1 = longitude1 / 180.0 * math.pi
    latitude1 = latitude1 / 180.0 * math.pi

    de = -1 * math.sin(longitude1) * (x2 - x1) + math.cos(longitude1) * (y2 - y1)
    dn = -1 * math.sin(latitude1) * math.cos(longitude1) * (x2 - x1) - math.sin(latitude1) * math.sin(longitude1) * (
                y2 - y1) + math.cos(latitude1) * (z2 - z1)
    du = math.cos(latitude1) * math.cos(longitude1) * (x2 - x1) + math.cos(latitude1) * math.sin(longitude1) * (
                y2 - y1) + math.sin(latitude1) * (z2 - z1)
    return [de, dn, du]


def lla2enu(start_longi, start_lati, start_h, longi, lati, h):
    x1, y1, z1 = lla2xyz(start_longi, start_lati, start_h)
    x2, y2, z2 = lla2xyz(longi, lati, h)
    de, dn, du = xyz2enu(start_longi, start_lati, x1, y1, z1, x2, y2, z2)
    return [de, dn, du]

# 创建一个经纬高坐标系统的转换器（WGS 84坐标系，单位为度和米）
# wgs84 = pyproj.Geod(ellps="WGS84")

# 初始化坐标文件名
output_file = "fix1.txt"

# 回调函数，处理GPS数据和姿态数据
def gps_callback(gps_msg):
    global output_file

    # 获取GPS数据
    longitude, latitude, altitude = gps_msg.longitude, gps_msg.latitude, gps_msg.altitude
    # longitude, latitude, altitude = gps_msg.longitude, gps_msg.latitude, gps_msg.height

    # # 获取姿态信息
    # roll, pitch, yaw = gps_msg.roll, gps_msg.pitch, gps_msg.azimuth

    # 将经纬高坐标转换为东北天坐标
    #east, north, up = wgs84.fwd(longitude, latitude, altitude, yaw, pitch, roll)
    [east, north, up] = lla2enu(116.154818803, 39.919378101, 73.248, longitude, latitude, altitude)
    # [east, north, up] = lla2enu(-74.56348466, 40.66349409666667, 38.381, longitude, latitude, altitude)#--->NOVATEL

    # 将数据写入文本文件
    with open(output_file, "a") as f:
        #以空格隔开
        f.write(f"{east} {north} {up}\n")

odom_file = "odom1.txt"
def odom_callback(msg):
    global x, y, z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    with open(odom_file, "a") as f:
        f.write(f"{x} {y} {z}\n")

if __name__ == '__main__':
    try:
        rospy.init_node('toenu')

        # 创建订阅者，订阅GPS数据和姿态数据
        #sub_gps = rospy.Subscriber('/jackal2/gps/fix', NavSatFix, gps_callback)
        sub_gps = rospy.Subscriber('/fixposition/navsatfix', NavSatFix, gps_callback)
        # rospy.Subscriber('/novatel/oem7/inspva', INSPVA, gps_callback)

        sub_odom = rospy.Subscriber("/robot/dlio/odom_node/odom", Odometry, odom_callback)
    

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
