#include <graph_slam/nmea_analyzer.h>

nmea_analyzer::nmea_analyzer()
{

}

nmea_analyzer::~nmea_analyzer()
{

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

void nmea_analyzer::analyze(const nmea_msgs::Sentence::ConstPtr &msg)
{
    current_time_ = msg->header.stamp;
    convert_sentence_(split_(msg->sentence), msg->header.stamp);
    return;
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