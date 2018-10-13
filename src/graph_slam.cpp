#include <graph_slam/graph_slam.h>

graph_slam::graph_slam() : tf_listener_(tf_buffer_)
{
    nh_.param<std::string>(ros::this_node::getName()+"/robot_frmae", robot_frmae_, "base_link");
    nh_.param<std::string>(ros::this_node::getName()+"/imu_frame", imu_frame_, "imu");
    nh_.param<std::string>(ros::this_node::getName()+"/gps_frame", gps_frame_, "gps");
    nh_.param<std::string>(ros::this_node::getName()+"/lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "map");
    nh_.param<bool>(ros::this_node::getName()+"/use_gps", use_gps_, true);
    pointcloud_sub_ = nh_.subscribe(ros::this_node::getName()+"/pointcloud",10, &graph_slam::pointcloud_callback_, this);
    imu_sub_ = nh_.subscribe(ros::this_node::getName()+"/imu",10, &graph_slam::imu_callback_, this);
    nmea_sub_ = nh_.subscribe(ros::this_node::getName()+"/nmea_sentence",10, &graph_slam::nmea_callback_, this);
    nmea_analyzer_ptr_ = std::make_shared<nmea_analyzer>(map_frame_);
}

graph_slam::~graph_slam()
{
    
}

void graph_slam::pointcloud_callback_(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    geometry_msgs::TransformStamped transform_stamped;
    sensor_msgs::PointCloud2 pointcloud_transformed;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(lidar_frame_, robot_frmae_ , ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Could NOT transform " << lidar_frame_ << " to " << robot_frmae_ << " " << ex.what());
        return;
    }
    return;
}

void graph_slam::imu_callback_(const sensor_msgs::Imu::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::Vector3Stamped linear_acc_stamped;
    linear_acc_stamped.vector = msg->linear_acceleration;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(imu_frame_, robot_frmae_ , ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Could NOT transform " << imu_frame_ << " to " << robot_frmae_ << " " << ex.what());
        return;
    }
    tf2::doTransform(linear_acc_stamped, linear_acc_stamped, transform_stamped);
    return;
}

void graph_slam::nmea_callback_(const nmea_msgs::Sentence::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return;
}