#ifndef GRAPH_SLAM_H_INCLUDED
#define GRAPH_SLAM_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>

//headers in STL
#include <mutex>

class graph_slam
{
public:
    graph_slam();
    ~graph_slam();
private:
    std::mutex sensor_mutex_;
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
    std::string map_frame_;
    void imu_callback_(const sensor_msgs::Imu::ConstPtr msg);
    void nmea_callback_(const nmea_msgs::Sentence::ConstPtr msg);
    void pointcloud_callback_(const sensor_msgs::PointCloud2::ConstPtr msg);
};

#endif  //GRAPH_SLAM_H_INCLUDED