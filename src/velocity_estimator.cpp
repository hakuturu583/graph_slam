#include <graph_slam/velocity_estimator.h>

velocity_estimator::velocity_estimator(double buffer_length, geometry_msgs::Vector3 initial_velocity) : buffer_(buffer_length)
{
    initial_velocity_ = initial_velocity;
    is_recieved_ = false;
}

velocity_estimator::velocity_estimator(double buffer_length) : buffer_(buffer_length)
{
    initial_velocity_.x = 0;
    initial_velocity_.y = 0;
    initial_velocity_.z = 0;
    is_recieved_ = false;
}

velocity_estimator::~velocity_estimator()
{

}

void velocity_estimator::input_linear_acceralation_value(geometry_msgs::Vector3Stamped linear_acceralation)
{
    if(is_recieved_ == false)
    {
        geometry_msgs::Vector3Stamped init_data;
        init_data.vector = initial_velocity_;
        init_data.header = linear_acceralation.header;
        buffer_.add_data(init_data);
    }
    else
    {
        
    }
    return;
}