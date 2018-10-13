#ifndef VELOCITY_ESTIMATOR_H_INCLUDED
#define VELOCITY_ESTIMATOR_H_INCLUDED

//headers in this package
#include <graph_slam/velocity_buffer.h>

//headers in ROS
#include <geometry_msgs/Vector3.h>

//headers in STL
#include <mutex>

//headers in boost
#include <boost/optional.hpp>

class velocity_estimator
{
public:
    velocity_estimator(double buffer_length, geometry_msgs::Vector3 initial_velocity);
    velocity_estimator(double buffer_length);
    ~velocity_estimator();
    void input_linear_acceralation_value(geometry_msgs::Vector3Stamped linear_acceralation);
private:
    std::mutex mtx_;
    volatile bool is_recieved_;
    geometry_msgs::Vector3 initial_velocity_;
    velocity_buffer buffer_;
};

#endif  //VELOCITY_ESTIMATOR_H_INCLUDED