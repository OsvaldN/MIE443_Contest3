#include <interaction.h>


void Interact(int emotion, ros::Publisher *vel_pub, std::string path_to_sounds, bool verbose){
    /*
    Input an integer of the type of motion
    0 = Angry
    1 = Disgust
    2 = Fear
    3 = Happy
    4 = Sad
    5 = Surprise
    6 = Neutral
    */

    sound_play::SoundClient sc;


    if (emotion == 0){ // Angry

        sc.playWave(path_to_sounds + "sound.wav"); // play Sound

        stepDistance(0.5, SPEED_LIM, vel_pub, true);
        stepDistance(0.5, SPEED_LIM, vel_pub, true, true);
        rotByAngle(-M_PI/6, vel_pub);
    }
    else if (emotion == 1){ // Disgust

    }
    else if (emotion == 2){ // Fear

    }
    else if (emotion == 3){ // Happy

    }
    else if (emotion == 4){ // Sad

    }
    else if (emotion == 5){ // Surprise

    }
    else if (emotion == 6){ // Neutral

    }

    else {
        if (verbose){
            ROS_INFO("Invalid emotion type");
        }
    }
}

