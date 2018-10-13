#include <graph_slam/velocity_buffer.h>

velocity_buffer::velocity_buffer(double buffer_length)
{

}

velocity_buffer::~velocity_buffer()
{

}

boost::optional<geometry_msgs::Vector3Stamped> velocity_buffer::query_newest_data()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time start_time = ros::Time::now() - ros::Duration(buffer_length_);
    std::vector<geometry_msgs::Vector3Stamped> new_buf_;
    for(auto itr = buf_.begin(); itr != buf_.end(); itr++)
    {
        if(itr->header.stamp > start_time)
        {
            new_buf_.push_back(*itr);
        }
    }
    buf_ = new_buf_;
    if(buf_.size() == 0)
    {
        ROS_WARN_STREAM("no data in the velocity buffer.");
        return boost::none;
    }
    return buf_[buf_.size()-1];
}

boost::optional<geometry_msgs::Vector3Stamped> velocity_buffer::query_data(ros::Time target_time)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time start_time = ros::Time::now() - ros::Duration(buffer_length_);
    std::vector<geometry_msgs::Vector3Stamped> new_buf_;
    for(auto itr = buf_.begin(); itr != buf_.end(); itr++)
    {
        if(itr->header.stamp > start_time)
        {
            new_buf_.push_back(*itr);
        }
    }
    buf_ = new_buf_;
    if(buf_.size() == 0)
    {
        ROS_WARN_STREAM("no data in the velocity buffer.");
        return boost::none;
    }
    if(buf_[0].header.stamp > target_time)
    {
        ROS_WARN_STREAM("requested time to the velocity buffer is too old.");
        return boost::none;
    }
    if(buf_[buf_.size()-1].header.stamp < target_time)
    {
        ROS_WARN_STREAM("requested time to the velocity buffer is in the future.");
        return boost::none;
    }
    for(int i=0; i<(buf_.size()-1); i++)
    {
        if(buf_[i].header.stamp < target_time && target_time < buf_[i+1].header.stamp)
        {
            geometry_msgs::Vector3Stamped ret;
            double dt1 = (target_time - buf_[i].header.stamp).toSec();
            double dt2 = (buf_[i+1].header.stamp - target_time).toSec();
            ret.vector.x = (buf_[i+1].vector.x * dt2 + buf_[i].vector.x * dt1)/(dt1 + dt2);
            ret.vector.y = (buf_[i+1].vector.y * dt2 + buf_[i].vector.y * dt1)/(dt1 + dt2);
            ret.vector.z = (buf_[i+1].vector.z * dt2 + buf_[i].vector.z * dt1)/(dt1 + dt2);
            return ret;
        }
    }
}
void velocity_buffer::add_data(geometry_msgs::Vector3Stamped data)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time start_time = ros::Time::now() - ros::Duration(buffer_length_);
    buf_.push_back(data);
    std::vector<geometry_msgs::Vector3Stamped> new_buf_;
    for(auto itr = buf_.begin(); itr != buf_.end(); itr++)
    {
        if(itr->header.stamp > start_time)
        {
            new_buf_.push_back(*itr);
        }
    }
    buf_ = new_buf_;
    return;
}