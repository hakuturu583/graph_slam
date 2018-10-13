#ifndef NMEA_ANALYZER_H_INCLUDED
#define NMEA_ANALYZER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <graph_slam/nmea_analyzer.h>
#include <graph_slam/geo_pos_conv.h>
#include <nmea_msgs/Sentence.h>
#include <tf2_ros/transform_broadcaster.h>

//headers in STL
#include <vector>
#include <string>
#include <sstream>

class nmea_analyzer
{
public:
    nmea_analyzer();
    ~nmea_analyzer();
    void analyze(const nmea_msgs::Sentence::ConstPtr &msg);
private:
    std::vector<std::string> split_(const std::string &string);
    void convert_sentence_(std::vector<std::string> nmea, ros::Time current_stamp);
    double orientation_time_, position_time_;
    ros::Time current_time_, orientation_stamp_;
    double roll_, pitch_, yaw_;
    geo_pos_conv geo_;
    geo_pos_conv last_geo_;
};

#endif  //NMEA_ANALYZER_H_INCLUDED