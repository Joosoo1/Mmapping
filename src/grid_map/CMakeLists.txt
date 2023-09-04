cmake_minimum_required(VERSION 3.0.2)
project(grid_map)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_filters
  # message_generation
  sensor_msgs
  nav_msgs
  occ_grid
  mutil_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map
  CATKIN_DEPENDS roscpp rospy std_msgs occ_grid 
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(grid_map_node 
  src/grid_map_node.cpp
  src/grid_map.cpp
  src/converter.cpp)

target_link_libraries(grid_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


