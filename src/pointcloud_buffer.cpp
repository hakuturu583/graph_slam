#include <graph_slam/pointcloud_buffer.h>

pointcloud_buffer::pointcloud_buffer(double buffer_length)
{
    buffer_length_ = buffer_length;
}

pointcloud_buffer::~pointcloud_buffer()
{

}

void pointcloud_buffer::add_data(std::shared_ptr<sensor_msgs::PointCloud2> data)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time start_time = ros::Time::now() - ros::Duration(buffer_length_);
    buf_.push_back(data);
    std::vector<std::shared_ptr<sensor_msgs::PointCloud2> > new_buf;
    for(int i = 0; i<buf_.size(); i++)
    {
        if(buf_[i]->header.stamp > start_time)
        {
            new_buf.push_back(buf_[i]);
        }
    }
    buf_ = new_buf;
    return;
}

std::vector<std::shared_ptr<sensor_msgs::PointCloud2> > pointcloud_buffer::get_buffer()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return buf_;
}