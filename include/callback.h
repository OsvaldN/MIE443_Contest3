#ifndef __CALLBACK_H_INCLUDED__
#define __CALLBACK_H_INCLUDED__

#include "contest3.h"

// ==== FORWARD DECLARATION FOR CALLBACK FUNCTION VARIABLES ====
extern float angular;
extern float linear; 
extern float posX;
extern float posY;
extern float yaw;
extern uint8_t bumper[3];

// ==== CALLBACK FUNCTION DECLARATIONS ====== 
extern void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
extern void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

#endif