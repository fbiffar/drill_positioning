#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <memory.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/DrillPosition.h>
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <dynamic_reconfigure/server.h>
#include <drill_positioning/DrillPositioningConfig.h>
#include <drill_positioning/drill_positioning.h>

namespace drill_positioning
{

    class drill_positioningNode
    {
    public:
        drill_positioningNode(const ros::NodeHandle &nh, const ros::NodeHandle private_nh);
        ~drill_positioningNode();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        drill_positioning drill_positioning_;

        ros::Subscriber odometry_sub_;
        ros::Subscriber desried_position_subscriber_ ;
        ros::Subscriber measured_position_subscriber_ ;
        ros::Subscriber rc_subscriber_;

        ros::Publisher position_publisher_;
        ros::Publisher joint1_pub_;
        ros::Publisher joint2_pub_;

        bool got_first_position_command_;
        bool got_first_position_;

        



        void Desired_Position_Callback(const mav_msgs::DrillPosition &desired_position);
       
        void Measured_Position_Callback(const std_msgs::String &measured_position);

        void OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

        void RC_Callback(const sensor_msgs::Joy::ConstPtr &rc_message);
    

        //void DynConfigCallback(drill_positioning::DrillPositioningConfig &config, uint32_t level);

        //dynamic_reconfigure::Server<drill_positioning::DrillPositioningConfig> dyn_config_server_;
    };

}


