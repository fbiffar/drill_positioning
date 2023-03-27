#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <sensor_msgs/Joy.h>

namespace drill_positioning
{
    class drill_positioning
    {
    public:
        drill_positioning(const ros::NodeHandle &nh, const ros::NodeHandle private_nh);
        ~drill_positioning();

        void InitializeParams();

        void SetHinge(double hinge_1, double hinge_2)
        {
            servo = (hinge_1 < -0.9 and hinge_2 > -0.9);
        }

        void SetMeasuredPosition(std_msgs::String meas_pos)

        {
            std::string s = meas_pos.data;
            const char delim = '\n';
            std::vector<std::string> out;
            tokenize(s, delim, out);
            meas_position_ << std::stod(out.at(0)), std::stod(out.at(1));
            std::cout << "meas_dist" << meas_position_ << std::endl;
        }

        void SetDesiredPosition(double des_y, double des_z)
        {
            des_position_ << des_y, des_z;
        }

        void SetOdometry(const mav_msgs::EigenOdometry &odometry);

        void CalculateDesiredPosition(Eigen::Vector2d *maxon_position);

    private:
        void tokenize(std::string const &str, const char delim,
                      std::vector<std::string> &out)
        {
            // construct a stream from the string
            std::stringstream ss(str);

            std::string s;
            while (std::getline(ss, s, delim))
            {
                out.push_back(s);
            }
        }
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        bool initialized_params_;

        double des_y_;
        double des_z_;

        double meas_y_;
        double meas_z_;

        double y_offset_ = 0;
        double z_offset_ = 0;

        bool servo = true;

        Eigen::Vector2d des_position_;
        mav_msgs::EigenOdometry odometry_;
        Eigen::Vector2d meas_position_;
    };
}
