#include <graph_slam/nmea_analyzer.h>

nmea_analyzer::nmea_analyzer(std::string map_frame)
{ 
    map_frame_ = map_frame;
}

nmea_analyzer::~nmea_analyzer()
{

}

void nmea_analyzer::publish_transform_()
{
    geometry_msgs::TransformStamped transform_stamped_;
    transform_stamped_.transform.translation.x = geo_.y();
    transform_stamped_.transform.translation.y = geo_.x();
    transform_stamped_.transform.translation.z = geo_.z();
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll_, pitch_, yaw_);
    transform_stamped_.transform.rotation.x = quaternion.x();
    transform_stamped_.transform.rotation.y = quaternion.y();
    transform_stamped_.transform.rotation.z = quaternion.w();
    transform_stamped_.transform.rotation.w = quaternion.z();
    broadcaster_.sendTransform(transform_stamped_);
    return;
}

std::vector<std::string> nmea_analyzer::split_(const std::string &string)
{
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(string);
    while (getline(ss, token, ','))
        str_vec_ptr.push_back(token);
    return str_vec_ptr;
}

void nmea_analyzer::create_orientation_()
{
    yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
    roll_ = 0;
    pitch_ = 0;
    return;
}

geometry_msgs::PoseStamped nmea_analyzer::get_current_pose_()
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = current_time_;
    pose.pose.position.x = geo_.y();
    pose.pose.position.y = geo_.x();
    pose.pose.position.z = geo_.z();
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll_, pitch_, yaw_);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.w();
    pose.pose.orientation.w = quaternion.z();
    return pose;
}

boost::optional<geometry_msgs::PoseStamped> nmea_analyzer::analyze(const nmea_msgs::Sentence::ConstPtr &msg)
{
    current_time_ = msg->header.stamp;
    convert_sentence_(split_(msg->sentence), msg->header.stamp);
    double timeout = 10.0;
    if (fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
    {
        double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
        double threshold = 0.2;
        if (dt > threshold)
        {
            ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
            create_orientation_();
            publish_transform_();
            last_geo_ = geo_;
        }
        return get_current_pose_();
    }

    double e = 1e-2;
    if (fabs(orientation_time_ - position_time_) < e)
    {
        publish_transform_();
        return get_current_pose_();
    }
    return boost::none;
}

void nmea_analyzer::convert_sentence_(std::vector<std::string> nmea, ros::Time current_stamp)
{
    try
    {
        if (nmea.at(0).compare(0, 2, "QQ") == 0)
        {
            orientation_time_ = stod(nmea.at(3));
            roll_ = stod(nmea.at(4)) * M_PI / 180.;
            pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
            yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
            orientation_stamp_ = current_stamp;
            ROS_INFO("QQ is subscribed.");
        }
        else if (nmea.at(0) == "$PASHR")
        {
            orientation_time_ = stod(nmea.at(1));
            roll_ = stod(nmea.at(4)) * M_PI / 180.;
            pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
            yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
            ROS_INFO("PASHR is subscribed.");
        }
        else if(nmea.at(0).compare(3, 3, "GGA") == 0)
        {
            position_time_ = stod(nmea.at(1));
            double lat = stod(nmea.at(2));
            double lon = stod(nmea.at(4));
            double h = stod(nmea.at(9));
            geo_.set_llh_nmea_degrees(lat, lon, h);
            ROS_INFO("GGA is subscribed.");
        }
        else if(nmea.at(0) == "$GPRMC")
        {
            position_time_ = stoi(nmea.at(1));
            double lat = stod(nmea.at(3));
            double lon = stod(nmea.at(5));
            double h = 0.0;
            geo_.set_llh_nmea_degrees(lat, lon, h);
            ROS_INFO("GPRMC is subscribed.");
        }
    }catch (const std::exception &e)
        {
            ROS_WARN_STREAM("Message is invalid : " << e.what());
        }
    return;
}