// headers in ROS
#include <ros/ros.h>

//headers in graph_slam
#include <graph_slam/graph_slam.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosbagraphslam_node");
    graph_slam slam;
    ros::spin();
    return 0;
}