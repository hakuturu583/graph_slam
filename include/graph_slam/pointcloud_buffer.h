#ifndef POINTCLOUD_BUFFER_H_INCLUDED
#define POINTCLOUD_BUFFER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//headers in STL
#include <vector>
#include <mutex>
#include <memory>

class pointcloud_buffer
{
public:
    pointcloud_buffer(double buffer_length);
    ~pointcloud_buffer();
    void add_data(std::shared_ptr<sensor_msgs::PointCloud2> data);
    std::vector<std::shared_ptr<sensor_msgs::PointCloud2> > get_buffer();
private:
    double buffer_length_;
    std::mutex mtx_;
    std::vector<std::shared_ptr<sensor_msgs::PointCloud2> > buf_;
};
#endif  //POINTCLOUD_BUFFER_H_INCLUDED