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
        boost::optional<geometry_msgs::Vector3Stamped> newest_data = buffer_.query_newest_data();
        if(newest_data)
        {
            geometry_msgs::Vector3Stamped input_data = newest_data.get();
            input_data.vector.x = input_data.vector.x + linear_acceralation.vector.x;
            input_data.vector.y = input_data.vector.y + linear_acceralation.vector.y;
            input_data.vector.z = input_data.vector.z + linear_acceralation.vector.z;
            buffer_.add_data(input_data);
        }
    }
    return;
}