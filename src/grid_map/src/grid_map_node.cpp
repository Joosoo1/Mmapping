#include "grid_map/grid_map.h"



int main(int argc, char **argv){
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh("~");
    
    GridMap2_5D grid_map(nh);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    grid_map.start();
    ros::waitForShutdown();

    return 0;
}