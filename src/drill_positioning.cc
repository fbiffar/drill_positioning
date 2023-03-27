#include <drill_positioning/drill_positioning.h>

namespace drill_positioning
{

    drill_positioning::drill_positioning(const ros::NodeHandle &nh, const ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          initialized_params_(false)
    {
    }

    drill_positioning::~drill_positioning()
    {
    }

    void drill_positioning::SetOdometry(const mav_msgs::EigenOdometry &odometry)
    {
        odometry_ = odometry;
    }

    void drill_positioning::CalculateDesiredPosition(Eigen::Vector2d *maxon_position){

        Eigen::Vector3d current_rpy;
        odometry_.getEulerAngles(&current_rpy);
        current_rpy(0) = 0.785;
        double distance_up; 
        double distance_right;

        distance_right = cos(current_rpy(0)) * (y_offset_ + meas_position_(0)); //cancel roll
        

        if (true){   //servo open or drone in upright position and servo locked 
                                                      
            distance_up = cos(current_rpy(0)) * meas_position_(1) - sin(current_rpy(0)) * y_offset_;

        }
        else{ //in flight cancelling pitch 

            distance_up = cos(current_rpy(1)) * (cos(current_rpy(0)) * meas_position_(1) - sin(current_rpy(0)) * z_offset_) - sin(current_rpy(1)) * z_offset_;

        }

        std::cout<<"positions y :"<< distance_right<<std::endl;
        std::cout<<"positions z :"<< distance_up<<std::endl;

        double y_pos_error = des_y_ - distance_right;
        double z_pos_error = des_z_ - distance_up;

        double Joint1 = z_pos_error + y_pos_error;
        double Joint2 = z_pos_error - y_pos_error; 

        *maxon_position << Joint1, Joint2; 
    }

    void drill_positioning::InitializeParams()
    {
        initialized_params_ = true;
    }

}