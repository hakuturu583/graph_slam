#ifndef GRAPH_SLAM_H_INCLUDED
#define GRAPH_SLAM_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class graph_slam
{
public:
    graph_slam();
    ~graph_slam();
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    std::string pointcloud_topic_name_;
    ros::Subscriber nmea_sub_;
    std::string nmea_topic_;
    ros::Subscriber imu_sub_;
    std::string imu_topic_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string robot_frmae_;
    std::string imu_frame_;
    std::string gps_frame_;
    std::string lidar_frame_;
};

#endif  //GRAPH_SLAM_H_INCLUDED