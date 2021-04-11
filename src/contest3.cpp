

#include "contest3.h"

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

    // The code below shows how to start and stop frontier exploration.

    explore.stop();
    explore.start();
    while(ros::ok()) {
        // Your code here.

    
        Interact(0, &vel_pub, true);
        Interact(1, &vel_pub, true);
        Interact(2, &vel_pub, true);
        Interact(3, &vel_pub, true);
        Interact(4, &vel_pub, true);
        Interact(5, &vel_pub, true);
        Interact(6, &vel_pub, true);

        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
