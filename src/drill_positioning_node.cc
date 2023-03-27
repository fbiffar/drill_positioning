#include <mav_msgs/default_topics.h>
#include <std_msgs/Float64.h>
#include <drill_positioning/drill_positioning_node.h>

namespace drill_positioning
{

    drill_positioningNode::drill_positioningNode(const ros::NodeHandle &nh,
                                           const ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          drill_positioning_(nh, private_nh)
    {

        drill_positioning_.InitializeParams();

        // subscriptions

        desried_position_subscriber_= nh_.subscribe(
                            mav_msgs::default_topics::COMMAND_DRILL_POSITION, 1, &drill_positioningNode::Desired_Position_Callback, this);

        //measured_position_subscriber_= nh_.subscribe(
        //                              mav_msgs::default_topics::COMMAND_DRILL_POSITION, 1, &drill_positioningNode::Measured_Position_Callback, this);
        
        measured_position_subscriber_ = nh_.subscribe("read", 100, &drill_positioningNode::Measured_Position_Callback, this);

        odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                      &drill_positioningNode::OdometryCallback, this,
                                      ros::TransportHints().tcpNoDelay());

        // for RC_input
        rc_subscriber_ = nh_.subscribe<sensor_msgs::Joy>(
                "input/rc", 100,&drill_positioningNode::RC_Callback, this);

        // publications
        //needs to be implemented

       // position_publisher_ = nh_.advertise<mav_msgs::DrillPosition>(mav_msgs::default_topics::COMMAND_DRILL_POSITION, 1);

        joint1_pub_ = nh_.advertise<std_msgs::Float64>("/maxon/canopen_motor/base_link1_joint_position_controller", 1);

        joint2_pub_ = nh_.advertise<std_msgs::Float64>("/maxon/canopen_motor/base_link2_joint_position_controller", 1);

        /*dynamic_reconfigure::Server<drill_positioning::DrillPositioningConfig>::CallbackType f;
        f = boost::bind(&drill_positioningNode::DynConfigCallback, this, _1, _2);
        dyn_config_server_.setCallback(f);*/
    }

    drill_positioningNode::~drill_positioningNode()
    {
    }

    void drill_positioningNode::Desired_Position_Callback(const mav_msgs::DrillPosition &desired_position)
    {

       drill_positioning_.SetDesiredPosition(desired_position.y_pos,
                                             desired_position.z_pos);

        got_first_position_command_ = true;

    }

    void drill_positioningNode::Measured_Position_Callback(const std_msgs::String &measured_position)
    {

       drill_positioning_.SetMeasuredPosition(measured_position);

        got_first_position_ = true;

    }


    void drill_positioningNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
    {

       // if (got_first_position_command_ == false or got_first_position_ == false)
       //     return;

        mav_msgs::EigenOdometry odometry;
        eigenOdometryFromMsg(*odometry_msg, &odometry);

        drill_positioning_.SetOdometry(odometry);

        Eigen::Vector2d maxon_position;
        drill_positioning_.CalculateDesiredPosition(&maxon_position);

        std_msgs::Float64 Joint1;
        std_msgs::Float64 Joint2;
        
        Joint1.data = maxon_position(0);
        Joint2.data = maxon_position(1);
     
        joint1_pub_.publish(Joint1);
        joint2_pub_.publish(Joint2);

    }

    void drill_positioningNode::RC_Callback(const sensor_msgs::Joy::ConstPtr &rc_message)
    {
        drill_positioning_.SetHinge(rc_message->axes.at(7), rc_message->axes.at(8));
    }

   /* void drill_positioningNode::DynConfigCallback(drill_positioning::DrillPositioningConfig &config,
                                               uint32_t level)
    {

     
    }*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drill_positioningNode");

    ros::NodeHandle nh, private_nh("~");
    drill_positioning::drill_positioningNode drill_positioning(nh, private_nh);

    ros::spin();

    return 0;
}