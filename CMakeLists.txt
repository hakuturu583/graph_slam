cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES graph_slam
  CATKIN_DEPENDS geometry_msgs nmea_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(graph_slam_node_src
  src/graph_slam_node.cpp 
  src/graph_slam.cpp 
  src/nmea_analyzer.cpp 
  src/geo_pos_conv.cpp
  src/velocity_estimator.cpp
  src/velocity_buffer.cpp
)
add_executable(graph_slam_node ${graph_slam_node_src})
target_link_libraries(graph_slam_node ${catkin_LIBRARIES})

install(TARGETS graph_slam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)