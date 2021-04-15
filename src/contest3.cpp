

#include "contest3.h"
#include "control.h"
#include <std_msgs/Int32.h> 

using namespace std;

//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play

#include <ros/console.h>

#define OutputFileName "Output_file.txt"
#define EMOTION_NONE -1
uint32_t emotion = EMOTION_NONE;

void emotionCallback(const std_msgs::Int32::ConstPtr &msg) {
    std::cout << "Emotion: " << msg->data << "\n\n";
    emotion = msg->data;
    return;
}

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

    // Subscribe to emotion classifier
    ros::Subscriber emotion_sub = n.subscribe("detected_emotion", 1, &emotionCallback);


    sound_play::SoundClient sc; // client for sound

    // Used to make the output file
    // Make and overwrite any files 
   
    ofstream myfile;
    myfile.open (OutputFileName);
    myfile << "This is the output file which will contain the output for Contest 3 run.\n";
    myfile.close();

    std::string Emotions[7] = {"Angry",
                              "Disgust",
                              "Fear",
                              "Happy",
                              "Sad",
                              "Surprise",
                              "Neutral"};

    // Start exploration 
    int return_val = rotByAngle(M_PI, &vel_pub, true); // Start with a spin to initiate exploration functionality
    explore.start();

    while(ros::ok()) {

        if (emotion != EMOTION_NONE) { // Victim is located
            explore.stop(); // Once a victim is found, stop exploration 

            std::cout << "Detected emotion: " << emotion << "\n\n"; // DEBUG REMOVE
            std::cout << "Detected emotion: " << Emotions[emotion] << "\n\n"; // DEBUG REMOVE
            Interact(emotion, &vel_pub, true);

            myfile.open(OutputFileName, std::ios_base::app); // append instead of overwrite
            myfile << "The predicted emotion of the victim found is: " << Emotions[emotion] << "\n";
            myfile.close();
            
            // Reset emotion parameters and resume exploration
            emotion = EMOTION_NONE;
            explore.start(); // Re-continue exploration
        }

        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
