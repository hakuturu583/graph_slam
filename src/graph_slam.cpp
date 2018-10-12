#include <graph_slam/graph_slam.h>

graph_slam::graph_slam() : tf_listener_(tf_buffer_)
{
    nh_.param<std::string>(ros::this_node::getName()+"/robot_frmae", robot_frmae_, "base_link");
    nh_.param<std::string>(ros::this_node::getName()+"/imu_frame", imu_frame_, "imu");
    nh_.param<std::string>(ros::this_node::getName()+"/gps_frame", gps_frame_, "gps");
    nh_.param<std::string>(ros::this_node::getName()+"/lidar_frame", lidar_frame_, "velodyne");
}

graph_slam::~graph_slam()
{
    
}