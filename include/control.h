#ifndef __HELPER_H_INCLUDED__
#define __HELPER_H_INCLUDED__

#include "contest3.h"

// ==== CONTROL HELPER FUNCTION DECLARATIONS ====== 
extern float randRange(float low, float high);
extern float angleCorrect(float curr, float start);
extern void VelPub(float angular, float linear, ros::Publisher *vel_pub);
extern int rotByAngle(float angle, ros::Publisher *vel_pub, bool verbose=false);
extern float eucDist(float x1, float y1, float x2, float y2);
extern float GetYawAngle(float input_angle, bool verbose=false);
extern int stepDistance(float distance, float speed,ros::Publisher *vel_pub, bool verbose=false, bool reverse=false);

#endif