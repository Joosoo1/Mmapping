#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "novatel_oem7_msgs/CORRIMU.h"
#include "novatel_oem7_msgs/INSPVAX.h"

ros::Publisher imu_pub;

void corrimucallback(const novatel_oem7_msgs::CORRIMU::ConstPtr& msg)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.angular_velocity.x = msg->roll_rate;
    imu_msg.angular_velocity.y = msg->pitch_rate;
    imu_msg.angular_velocity.z = msg->yaw_rate;
    imu_msg.linear_acceleration.x = msg->lateral_acc;
    imu_msg.linear_acceleration.y = msg->vertical_acc;
    imu_msg.linear_acceleration.z = msg->longitudinal_acc;
    imu_pub.publish(imu_msg);
    //ROS_INFO("Published IMU message");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trans2imu");
    ros::NodeHandle nh;
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Subscriber imu_sub = nh.subscribe("/novatel/oem7/corrimu", 1000, &corrimucallback);
    ros::spin();
    return 0;
}