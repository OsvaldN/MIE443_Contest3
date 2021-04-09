

#include "contest3.h"
#include "control.h"

//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play

#include <ros/console.h>

int main(int argc, char** argv) {
    //
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;


    ros::Subscriber bumper_sub = n.subscribe("mobile_base/events/bumper", 10, &bumperCallback);

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;
    //
    // Frontier exploration algorithm.
    explore::Explore explore;
    //


    sound_play::SoundClient sc; // client for sound

    // Start exploration 
    int return_val;
    return_val = rotByAngle(M_PI, &vel_pub, true); // Start with a spin to initiate exploration functionality
    explore.start();

    while(ros::ok()) {

        /** NOTE: Add the check for victim locator here from subscribed topic
        if (victim_located) {
            explore.stop(); // Stop exploration

        /** DEBUG REMOVE: @David's interaction code is found below.
         * Ask Oz's model for the image type, and then pass that into Interact function
            Interact(0, &vel_pub, path_to_sounds, true); // DEBUG REMOVE
            explore.start(); // Re-continue exploration
        }
        **/

        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
