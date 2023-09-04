#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/NavSatFix.h>

#include "grid_map/converter.h"
#include "mutil_msgs/MutilMapping.h"
#include "occ_grid/raycast.h"


#include <memory>
#include <random>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

class GridMap2_5D
{
public:
    GridMap2_5D(ros::NodeHandle node_handle);
    ~GridMap2_5D();
    void start();

private:
    void init();
    void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::OdometryConstPtr& odom);
    void rtkCallback(const sensor_msgs::NavSatFixConstPtr &rtk_self_msg, const sensor_msgs::NavSatFixConstPtr &rtk_world_msg);
    
    
    void initRobotStartPos(const sensor_msgs::NavSatFixConstPtr &rtk_self_msg, const sensor_msgs::NavSatFixConstPtr &rtk_world_msg);
    void recordRobotPosition(const std::vector<float> &robot_rtk_pos);

    
    void initPoseCallback(const nav_msgs::OdometryConstPtr &odom);
    void updateGridMap(const nav_msgs::OdometryConstPtr &odom);
    void gridGlobalTimerCallback(const ros::TimerEvent &event);
    void gridLocalVisTimerCallback(const ros::TimerEvent &event);
    

    void publishGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const Eigen::Vector3d& sensor_pos);
    void initPose();
    void intputPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const int& point_num, 
                        const Eigen::Vector3d& sensor_pos);
    // int findMaxZ(const std::vector<pcl::PointXYZ>& points);
    int findMaxZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const nav_msgs::MapMetaData &map_info, int cell_num);
    void raycast(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, const int& max_z,
                const Eigen::Vector3d& sensor_pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    
    //同步订阅agent位姿以及点云
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicyCloudOdom;
    typedef shared_ptr<message_filters::Synchronizer<syncPolicyCloudOdom>> SynchronizerCloudOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> syncPolicyRTK;
    typedef shared_ptr<message_filters::Synchronizer<syncPolicyRTK>> SynchronizerRTK;
    ros::NodeHandle nh_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> rtk_self_sub_, rtk_world_sub_;
    SynchronizerCloudOdom sync_cloud_odom_;
    SynchronizerRTK sync_rtk_;

    //创建话题发布者
    ros::Publisher grid_local_map_pub_, grid_map_vis_pub_, grid_global_map_pub_, mutil_msg_pub_;

    //经纬度转东北天
    Converter converter_;

    //光线投影器
    RayCaster ray_caster_;

    ros::Publisher grid_map_pub_;

    //小车上传感器的位置
    Eigen::Vector3d sensor_pos_;
    Eigen::Quaterniond sensor_ori_;

    //是否完成初始化标志位
    bool init_flag_;

    //传感器位置
    Eigen::Vector3d robot_start_pos_;
    Eigen::Quaterniond robot_start_ori_;

    //2.5D grid map 参数
    double map_resolution_;

    //局部栅格地图大小,以robot为中心的小方形？
    int map_local_size_x_, map_local_size_y_;

    //滤除超过限度的点云参数
    double min_height_, max_height_;

    //点云下采样参数
    double downsample_resolution_;
    int downsample_num_;

    //世界坐标系
    std::string world_frame_id_, ns;

    // //robot信息
    // int world_frame_robot_id;

    //创建定时器，用于定时更新全局栅格地图和局部三维地图可视化
    ros::Timer grid_global_timer_, grid_local_vis_timer_;

    //随机噪声
    normal_distribution<double> rand_noise_;
    default_random_engine rand_engine_;
};



#endif