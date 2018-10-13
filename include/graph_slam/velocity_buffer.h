#ifndef VELOCITY_BUFFER_H_INCLUDED
#define VELOCITY_BUFFER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

//headers in STL
#include <vector>
#include <mutex>

//headers in boost
#include <boost/optional.hpp>

class velocity_buffer
{
public:
    velocity_buffer(double buffer_length);
    ~velocity_buffer();
    void add_data(geometry_msgs::Vector3Stamped data);
    boost::optional<geometry_msgs::Vector3Stamped> query_newest_data();
    boost::optional<geometry_msgs::Vector3Stamped> query_data(ros::Time target_time);
private:
    std::mutex mtx_;
    double buffer_length_;
    std::vector<geometry_msgs::Vector3Stamped> buf_;
};

#endif  //VELOCITY_BUFFER_H_INCLUDED