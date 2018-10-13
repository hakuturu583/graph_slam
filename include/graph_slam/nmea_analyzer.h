#ifndef NMEA_ANALYZER_H_INCLUDED
#define NMEA_ANALYZER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <graph_slam/nmea_analyzer.h>
#include <graph_slam/geo_pos_conv.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

//headers in STL
#include <vector>
#include <string>
#include <sstream>

//headers in boost
#include <boost/optional.hpp>

class nmea_analyzer
{
public:
    nmea_analyzer(std::string map_frame);
    ~nmea_analyzer();
    boost::optional<geometry_msgs::PoseStamped> analyze(const nmea_msgs::Sentence::ConstPtr &msg);
private:
    void publish_transform_();
    void create_orientation_();
    geometry_msgs::PoseStamped get_current_pose_();
    std::vector<std::string> split_(const std::string &string);
    void convert_sentence_(std::vector<std::string> nmea, ros::Time current_stamp);
    double orientation_time_, position_time_;
    ros::Time current_time_, orientation_stamp_;
    double roll_, pitch_, yaw_;
    geo_pos_conv geo_;
    geo_pos_conv last_geo_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::string map_frame_;
};

#endif  //NMEA_ANALYZER_H_INCLUDED