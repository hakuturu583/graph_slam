#include <graph_slam/velocity_estimator.h>

velocity_estimator::velocity_estimator(double buffer_length, geometry_msgs::Vector3 initial_velocity) : buffer_(buffer_length)
{

}

velocity_estimator::velocity_estimator(double buffer_length) : buffer_(buffer_length)
{

}

velocity_estimator::~velocity_estimator()
{

}