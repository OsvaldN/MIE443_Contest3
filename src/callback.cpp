#include "callback.h"

/*
/////////////////////////////////
/ ROS callback helper functions /
/////////////////////////////////
*/


//variables for rotational speed and linear speed
float angular = 0.0;
float linear = 0.0; 

//odom variables
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

//bumper variables
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

/*
//////////////////////
/ Callback functions /
//////////////////////
*/

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT,CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}