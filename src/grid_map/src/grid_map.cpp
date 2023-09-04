#include "grid_map/grid_map.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

GridMap2_5D::GridMap2_5D(ros::NodeHandle node_handle) : nh_(node_handle){
    this->init();
}
GridMap2_5D::~GridMap2_5D(){}

void GridMap2_5D::init(){
    this->nh_.param("grid_map_2_5d/resolution", this->map_resolution_, 0.1);
    this->nh_.param("grid_map_2_5d/map_local_size_x", this->map_local_size_x_, 10);
    this->nh_.param("grid_map_2_5d/map_local_size_y", this->map_local_size_y_, 10);
    this->nh_.param("grid_map_2_5d/min_height", this->min_height_, 0.0);
    this->nh_.param("grid_map_2_5d/max_height", this->max_height_, 1.0);
    this->nh_.param("grid_map_2_5d/downsample_resolution", this->downsample_resolution_, 0.1);
    this->nh_.param("grid_map_2_5d/world_frame_id", this->world_frame_id_, std::string("world"));
    // nh_.param("grid_map_2_5d/world_frame_robot_id", this->world_frame_robot_id, 1);

    this->ns = ros::this_node::getNamespace();
    this->ns.erase(0, 1);
    std::cout << "ns: " << this->ns << std::endl;
    this->init_flag_ = false;

    // rand_noise_ = normal_distribution<double>(0, 0.1);
    // random_device rd;
    // rand_engine_ = default_random_engine(rd());

    this->grid_global_timer_ = this->nh_.createTimer(ros::Duration(0.1), &GridMap2_5D::gridGlobalTimerCallback, this);
    this->grid_local_vis_timer_ = this->nh_.createTimer(ros::Duration(0.1), &GridMap2_5D::gridLocalVisTimerCallback, this);

    //发布者名称,不同机器人不同
    // std::string grid_local_map_topic = "/robot" + std::to_string(robot_id) + "/grid2.5d/grid_local_map";
    // std::string grid_global_map_topic = "/robot" + std::to_string(robot_id) + "/grid2.5d/grid_global_map";
    // std::string grid_map_vis_topic =  "/robot" + std::to_string(robot_id) + "/grid2.5d/grid_map_vis";

    this->grid_local_map_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>("grid_local_map", 1, true);
    this->grid_global_map_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>( "grid_global_map", 1, true);
    this->grid_map_vis_pub_ = this->nh_.advertise<sensor_msgs::PointCloud2>( "grid_map_vis", 1, true);
    this->mutil_msg_pub_ = this->nh_.advertise<mutil_msgs::MutilMapping>( "mutil_msg", 1, true);

    // std::string cloud_topic = "/robot"+ std::to_string(robot_id) + cloud_topic_;
    // std::string odom_topic = "/robot" + std::to_string(robot_id) + odom_topic_;

    this->cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "pointcloud", 1, ros::TransportHints().tcpNoDelay()));
    this->odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 1, ros::TransportHints().tcpNoDelay()));

    this->sync_cloud_odom_.reset(new message_filters::Synchronizer<GridMap2_5D::syncPolicyCloudOdom>(GridMap2_5D::syncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    this->sync_cloud_odom_->registerCallback(boost::bind(&GridMap2_5D::cloudOdomCallback, this, _1, _2));

    //同步订阅自身机器人和主机器人rtk数据，以确定初始位姿pose
    // std::string rtk_self_topic = "/robot" + std::to_string(robot_id) + rtk_topic_;
    // std::string rtk_world_topic = "/robot" + std::to_string(robot_id) + rtk_topic_;
    this->rtk_self_sub_.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh_, "rtk_self", 1, ros::TransportHints().tcpNoDelay()));
    this->rtk_world_sub_.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh_, "rtk_world", 1, ros::TransportHints().tcpNoDelay()));
    this->sync_rtk_.reset(new message_filters::Synchronizer<GridMap2_5D::syncPolicyRTK>(GridMap2_5D::syncPolicyRTK(100), *rtk_self_sub_, *rtk_world_sub_));
    this->sync_rtk_->registerCallback(boost::bind(&GridMap2_5D::rtkCallback, this, _1, _2));
}

void GridMap2_5D::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const nav_msgs::OdometryConstPtr &odom_msg){
    std::stringstream ss1, ss2;
    // 如果没有初始化完成，进行初始化
    if (!this->init_flag_){
        ss1 << "obot" << this->ns << " is initializing";
        ROS_WARN("%s", ss1.str().c_str());
        return ;
    }

    // 初始化完成后显示消息
    ss2 << "\033[1;32mrobot" << this->ns << " has initialized!\033[0m";
    ROS_INFO("%s", ss2.str().c_str());

    // 获取传感器位置
    this->sensor_pos_(0) = odom_msg->pose.pose.position.x + this->robot_start_pos_(0);
    this->sensor_pos_(1) = odom_msg->pose.pose.position.y + this->robot_start_pos_(1);
    this->sensor_pos_(2) = odom_msg->pose.pose.position.z + this->robot_start_pos_(2);

    //获取传感器方向，并与初始方向相乘
    // this->sensor_ori_ = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation
    //                                  .x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation
    //                                  .z) * this->robot_start_ori_;
    this->sensor_ori_ = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
                                            odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

    Eigen::Matrix3d sensor_ori = sensor_ori_.toRotationMatrix();

    // 转换点云消息为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);


    //点云滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(this->downsample_resolution_, this->downsample_resolution_, this->downsample_resolution_);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*cloud_filtered);

    //TODO:雷达到imu(beselink)的转换矩阵Til
    //构建点云转换矩阵，将传感器坐标系下的点云，转到世界坐标系下
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = sensor_ori.cast<float>();
    transform_matrix.block<3, 1>(0, 3) = sensor_pos_.cast<float>();

    //创建滤波器，滤除离群点
    pcl::transformPointCloud(*cloud_filtered, *cloud_trans, transform_matrix);
    int num = cloud_trans->points.size();
    std::cout << "cloud_trans->points.size() = " << num << std::endl;

    this->intputPointcloud(cloud_trans, num, this->sensor_pos_);
    this->publishGridMap(cloud_trans, sensor_pos_);
}


void GridMap2_5D::rtkCallback(const sensor_msgs::NavSatFixConstPtr &rtk_self_msg, const sensor_msgs::NavSatFixConstPtr &rtk_world_msg) {
    if (!this->init_flag_) {
        // 第一次回调时初始化机器人起始位置
        this->initRobotStartPos(rtk_self_msg, rtk_world_msg);
        this->init_flag_ = true;
    }

    // 获取机器人在RTK定位下的东北天坐标系位置
    std::vector<float> robot_rtk_pos = converter_.lla2enu(rtk_self_msg);

    // 将机器人位置信息记录到文本文件中
    this->recordRobotPosition(robot_rtk_pos);
}

void GridMap2_5D::initRobotStartPos(const sensor_msgs::NavSatFixConstPtr &rtk_self_msg, const sensor_msgs::NavSatFixConstPtr &rtk_world_msg) {
    // 获取机器人在RTK定位下的初始位置
    this->converter_.convert_init(rtk_world_msg->longitude, rtk_world_msg->latitude, rtk_world_msg->altitude);
    std::vector<float> robot_rtk_pos = converter_.lla2enu(rtk_self_msg);

    // 将初始位置信息记录到类成员变量中
    this->robot_start_pos_(0) = robot_rtk_pos[0];
    this->robot_start_pos_(1) = robot_rtk_pos[1];
    this->robot_start_pos_(2) = robot_rtk_pos[2];

    // 将初始位置信息记录到文本文件中
    this->recordRobotPosition(robot_rtk_pos);

    std::cout << this->ns << " start pos: " << this->robot_start_pos_(0) << " " << this->robot_start_pos_(1) << " " << this->robot_start_pos_(2) << std::endl;
}

void GridMap2_5D::recordRobotPosition(const std::vector<float> &robot_rtk_pos) {
    // 将机器人位置信息记录到文本文件中
    std::ofstream outfile;
    outfile.open("/home/fly-drone/Mmapping/src/grid_map/traj/" + this->ns + "_rtk_pos.txt", std::ios::app);
    outfile << robot_rtk_pos[0] << " " << robot_rtk_pos[1] << " " << robot_rtk_pos[2] << std::endl;
    outfile.close();
}

void GridMap2_5D::intputPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const int& point_num, 
                                const Eigen::Vector3d& sensor_pos){
    if (point_num == 0){
        return;
    }


    // 计算裁减框的最小点和最大点
    Eigen::Vector4f min_point(sensor_pos[0] - (this->map_local_size_x_ / 2),
                              sensor_pos[1] - (this->map_local_size_y_ / 2),
                              min_height_, 1.0);
    Eigen::Vector4f max_point(sensor_pos[0] + (this->map_local_size_x_ / 2),
                              sensor_pos[1] + (this->map_local_size_y_ / 2),
                              max_height_, 1.0);//最后的参数1.0为置信度，一般使用默认值1.0

    // 打印最小点和最大点坐标
    std::cout << "Min Point: " << min_point[0] << ", " << min_point[1] << ", " << min_point[2] << std::endl;
    std::cout << "Max Point: " << max_point[0] << ", " << max_point[1] << ", " << max_point[2] << std::endl;

    //创建局部区域box，滤除box以外的点云
    pcl::CropBox<pcl::PointXYZ> boxFilter;//点云裁减滤波器
    boxFilter.setMin(min_point);
    boxFilter.setMax(max_point);

    boxFilter.setInputCloud(cloud_in);
    boxFilter.filter(*cloud_in);//将裁减后的点云的储存回cloud_in
    std::cout << "cloud_in->points.size() = " << cloud_in->size() << std::endl;
    //创建滤波器，滤除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_in);
    std::cout << "cloud_in->points.size() = " << cloud_in->size() << std::endl;
    //滤除孤立点
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(2);
    outrem.filter(*cloud_in);
    std::cout << "cloud_in->points.size() = " << cloud_in->size() << std::endl;
    //得到最终用作点云转换为栅格的点云
}

void GridMap2_5D::publishGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const Eigen::Vector3d& sensor_pos) {
    // 创建栅格地图
    nav_msgs::OccupancyGrid grid_msg;
    mutil_msgs::MutilMapping mutil_map;
    grid_msg.header.frame_id = this->world_frame_id_;
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.info.resolution = this->map_resolution_;
    grid_msg.info.width = this->map_local_size_x_ / this->map_resolution_;
    grid_msg.info.height = this->map_local_size_y_ / this->map_resolution_;
    grid_msg.info.origin.position.x = sensor_pos[0] - (this->map_local_size_x_ / 2);
    grid_msg.info.origin.position.y = sensor_pos[1] - (this->map_local_size_y_ / 2);

    // 初始化栅格数据为-1
    grid_msg.data.assign(grid_msg.info.width * grid_msg.info.height, -1);

    // 将点云映射到栅格地图
    for (const auto& point : *cloud_in) {
        int col = static_cast<int>((point.x - grid_msg.info.origin.position.x) / this->map_resolution_);
        int row = static_cast<int>((point.y - grid_msg.info.origin.position.y) / this->map_resolution_);

        if (col >= 0 && col < grid_msg.info.width && row >= 0 && row < grid_msg.info.height) {
            int cell_idx = row * grid_msg.info.width + col;
            grid_msg.data[cell_idx] = 1;
        }
    }
    // 查找每个包含点云的栅格单元中的最大高度
    for (int i = 0; i < grid_msg.info.width * grid_msg.info.height; i++) {
        if (grid_msg.data[i] == 1) { 
            grid_msg.data[i] = this->findMaxZ(cloud_in, grid_msg.info, i);
        }
    }
    this->grid_local_map_pub_.publish(grid_msg);
    mutil_map.msgGrid = grid_msg;
    mutil_map.usMsgVehicleCol = static_cast<int>(grid_msg.info.width / 2);
    mutil_map.usMsgVehicleRow = static_cast<int>(grid_msg.info.height / 2);
    mutil_map.cGridColumnScale = this->map_resolution_;
    mutil_map.cGridRowScale = this->map_resolution_;
    mutil_map.uiMsgVehicleFixedX = sensor_pos[0];
    mutil_map.uiMsgVehicleFixedY = sensor_pos[1];
    this->mutil_msg_pub_.publish(mutil_map);
}

int GridMap2_5D::findMaxZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const nav_msgs::MapMetaData &map_info, int cell_num) {
    if (cell_num < 0 || cell_num >= static_cast<int>(map_info.width * map_info.height)) {
        return 0;
    }

    int max_z = 0;
    int j_start = (cell_num % static_cast<int>(map_info.width)) * map_info.resolution;
    int i_start = (cell_num / static_cast<int>(map_info.width)) * map_info.resolution;

    for (const auto& point : *cloud_in) {
        int j = static_cast<int>((point.x - map_info.origin.position.x) / map_info.resolution);
        int i = static_cast<int>((point.y - map_info.origin.position.y) / map_info.resolution);

        if (i == i_start && j == j_start && point.z > max_z) {
            max_z = point.z;
        }
    }
    return max_z;
}

void GridMap2_5D::start(){
    ROS_INFO("\033[1;32mMutil mapping is started!\033[0m");
    ROS_INFO("\033[1;32mWaiting for the initialization!\033[0m");

}

void GridMap2_5D::raycast(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, const int& max_z,
                const Eigen::Vector3d& sensor_pos, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
                
                }

void GridMap2_5D::initPoseCallback(const nav_msgs::OdometryConstPtr &odom){}

void GridMap2_5D::updateGridMap(const nav_msgs::OdometryConstPtr &odom){}

void GridMap2_5D::gridGlobalTimerCallback(const ros::TimerEvent &event){}

void GridMap2_5D::gridLocalVisTimerCallback(const ros::TimerEvent &event){}

void GridMap2_5D::initPose(){}