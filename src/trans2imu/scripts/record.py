#!/usr/bin/env python
#conding=utf-8

import rospy
from sensor_msgs.msg import NavSatFix
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry


gps_file = "gps0.txt"
def gps_callback(msg):
    global longitudel, latitudel, altitude
    longitudel = msg.longitude
    latitudel = msg.latitude
    altitude = msg.altitude
 
    with open(gps_file, "a") as f:
        f.write(f"{longitudel} {latitudel} {altitude}\n")


odom_file = "odom2.txt"
def odom_callback(msg):
    global x, y, z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    with open(odom_file, "a") as f:
        f.write(f"{x} {y} {z}\n")




if __name__ == "__main__":
    try:
        rospy.init_node("gps_node")
        sub = rospy.Subscriber("/fo", NavSatFix, gps_callback)
        # sub = rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback)
        sub = rospy.Subscriber("/robot/dlio/odom_node/odom", Odometry, odom_callback)
        # sub = rospy.Subscriber("/fixposition/odometry", Odometry, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass