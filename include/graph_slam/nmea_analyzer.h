#ifndef NMEA_ANALYZER_H_INCLUDED
#define NMEA_ANALYZER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <graph_slam/nmea_analyzer.h>
#include <graph_slam/geo_pos_conv.h>

class nmea_analyzer
{
public:
    nmea_analyzer();
    ~nmea_analyzer();
    void convert(std::vector<std::string> nmea, ros::Time current_stamp);
private:
    double orientation_time_, position_time_;
    ros::Time current_time_, orientation_stamp_;
    double roll_, pitch_, yaw_;
    geo_pos_conv geo_;
    geo_pos_conv last_geo_;
};

#endif  //NMEA_ANALYZER_H_INCLUDED