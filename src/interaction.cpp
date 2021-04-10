#include <interaction.h>


void Interact(int emotion, ros::Publisher *vel_pub, bool verbose){
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

    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/"; // Path to sound files
    sound_play::SoundClient sc; // client to play sounds

    std::string path_to_videos = ros::package::getPath("mie443_contest3") + "/videos/"; // Path to sound files
    std::string video_window_name = "Human, Robot Interface";

    if (emotion == 0){ // Angry

        stepDistance(0.5, SPEED_LIM, vel_pub, true);
        stepDistance(0.5, SPEED_LIM, vel_pub, true, true);
        rotByAngle(-M_PI/6, vel_pub);

        sc.playWave(path_to_sounds + "sound.wav"); // play Sound

        std::string current_video = "test_video.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);  

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

