#include <control.h>

/*
////////////////////////////
/ Control helper functions /
////////////////////////////
*/

// random number generator
float randRange(float low, float high){
    // Uniform random from interval [low, high]
    return low + ( (rand() / double(RAND_MAX)) * (high-low) );
}

float angleCorrect(float curr, float start){
    // corrects angle discontinuity in yaw
    float x = fabs(curr-start);
    float y = fabs(360+curr-start);
    float z = fabs(-360+curr-start);
    return x < y ? (x < z ? x : z) : (y < z ? y : z); // bootleg way to get min of the three
}

void VelPub(float angular, float linear, ros::Publisher *vel_pub){
    //Publish a velocity pair using using vel_pub
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub->publish(vel);
    return;
}

int rotByAngle(float angle, ros::Publisher *vel_pub, bool verbose){
    //Rotates the robot by (angle) radians about the z-axis

    ros::Rate loop_rate(10);
    // Rotate at maximum speed in direction of angle
    float dir = (angle > 0) - (angle < 0);
    float rotVel = dir * MAX_SPINRATE;

    ros::spinOnce();
    float startYaw = yaw;
    float lastYaw = yaw;
    
    // initiate queue of past yaws with infs
    std::queue<float> yawQueue;
    for (int i=0;i<5;i++) { yawQueue.push(std::numeric_limits<float>::infinity());}

    if (verbose){
        ROS_INFO("Rotating by %f radians at vel: %f", angle, rotVel);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    while (angleCorrect(yaw, startYaw) < fabs(angle)) {    
        // publish to update velocity, spin to update yaw (clears velocity)
        VelPub(rotVel, 0.0, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
	
        //update yawQueue and check distance travelled
        yawQueue.push(yaw);
        yawQueue.pop();
        // if last 5 spins don't meet 0.05rad threshold rotation then give up
        if (angleCorrect(yawQueue.front(), yawQueue.back()) < 0.05) {
            if (verbose) {
                ROS_INFO("Got stuck trying to turn after %f degrees", RAD2DEG(angleCorrect(yaw, startYaw)) );
            }
            return 1;
        }
    }

    return 0;
}

float eucDist(float x1, float y1, float x2, float y2){
    // returns euclidean distance between (x1,y1) and (x2,y2)
    return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2) );
}



int stepDistance(float distance, float speed,ros::Publisher *vel_pub, bool verbose, bool reverse) {
    /*
    Moves the robot forward (distance) units at (speed)
        stops if there is a collision
     */
    
    ros::Rate loop_rate(10);
    // get start values
    ros::spinOnce();
    float startX = posX, startY = posY;
    float minLaserDistance; // Check the min distance from the laser reading

    if (verbose){
        ROS_INFO("Stepping %f units at speed: %f", distance, speed);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    while ( (eucDist(startX, startY, posX, posY) < distance) && !(bumpersPressed()) ){
        // Check if obstacle infront of Robot before moving

        /*
        minLaserDistance = minDistance(15);
        if (minLaserDistance < MIN_LASER_DIST || std::isinf(minLaserDistance) || std::isnan(minLaserDistance)) {
            laserObstacleDirectionHandler(vel_pub, true); // DEBUG: Change verbosity to false to limit console messages
        }
        */

        // publish to update speed, spin to update pos (clears velocity)
        if (reverse) {
            VelPub(0.0, -speed, vel_pub);

        } else {
            VelPub(0.0, speed, vel_pub);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    if (bumpersPressed()){
	return 1;
    }

    return 0;
}