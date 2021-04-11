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

        if (verbose){
            ROS_INFO("Victim seems to be angry so the robot is afraid the victim may be dangerous");
        }
        
        stepDistance(0.1, SPEED_LIM, vel_pub, true, true);

        sc.playWave(path_to_sounds + "angry.wav"); // play Sound

        std::string current_video = "angry_fear.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);

        rotByAngle(-M_PI/24, vel_pub);
        rotByAngle(M_PI/12, vel_pub);
        rotByAngle(-M_PI/12, vel_pub);
        rotByAngle(M_PI/12, vel_pub);
        rotByAngle(-M_PI/24, vel_pub);  

        stepDistance(0.1, SPEED_LIM, vel_pub, true);

    }
    else if (emotion == 1){ // Disgust

        if (verbose){
            ROS_INFO("Victim seems to be disgusted so the robot is embarrassed of itself");
        }

        stepDistance(0.1, SPEED_LIM, vel_pub, true, true);

        sc.playWave(path_to_sounds + "disgust.wav"); // play Sound

        std::string current_video = "disgust_embarrassed.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);

        rotByAngle(-M_PI/12, vel_pub);
        rotByAngle(M_PI/15, vel_pub);
        rotByAngle(-M_PI/15, vel_pub);
        rotByAngle(M_PI/12, vel_pub);

        stepDistance(0.1, SPEED_LIM, vel_pub, true);

    }
    else if (emotion == 2){ // Fear

        if (verbose){
            ROS_INFO("Victim seems to be afraid so the robot is proud to show confidence that it will be ok");
        }

        sc.playWave(path_to_sounds + "fear.wav"); // play Sound

        std::string current_video = "fear_pride.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);

        rotByAngle(-M_PI/2, vel_pub);
        stepDistance(0.05, SPEED_LIM, vel_pub, true);
        rotByAngle(M_PI/2, vel_pub);
        rotByAngle(M_PI/2, vel_pub);
        stepDistance(0.1, SPEED_LIM, vel_pub, true);
        rotByAngle(-M_PI/2, vel_pub);
        rotByAngle(-M_PI/2, vel_pub);
        stepDistance(0.05, SPEED_LIM, vel_pub, true);
        rotByAngle(M_PI/2, vel_pub);  

    }
    else if (emotion == 3){ // Happy

        if (verbose){
            ROS_INFO("Victim seems to be happy so the robot is angry that he may have caused the fire");
        }

        rotByAngle(5*M_PI/12, vel_pub);
        rotByAngle(5*M_PI/12, vel_pub);

        sc.playWave(path_to_sounds + "happiness.wav"); // play Sound

        std::string current_video = "happiness_rage.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);
        
        stepDistance(0.1, SPEED_LIM, vel_pub, true);
        rotByAngle(M_PI/3, vel_pub);
        rotByAngle(M_PI/3, vel_pub);
        stepDistance(0.1, SPEED_LIM, vel_pub, true);
        rotByAngle(M_PI/3, vel_pub);
        rotByAngle(M_PI/3, vel_pub);
        stepDistance(0.1, SPEED_LIM, vel_pub, true);
        rotByAngle(-M_PI/6, vel_pub);  

    }
    else if (emotion == 4){ // Sad

        if (verbose){
            ROS_INFO("Victim seems to be sad so the robot is positively excited to try to boost spirits");
        }
        
        stepDistance(0.05, SPEED_LIM, vel_pub, true);
        stepDistance(0.05, SPEED_LIM, vel_pub, true, true);
        sc.playWave(path_to_sounds + "sadness.wav"); // play Sound

        std::string current_video = "sadness_excited.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);

        rotByAngle(M_PI/2, vel_pub);
        rotByAngle(M_PI/2, vel_pub);
        rotByAngle(M_PI/2, vel_pub);
        rotByAngle(M_PI/2, vel_pub);
        stepDistance(0.05, SPEED_LIM, vel_pub, true);
        stepDistance(0.05, SPEED_LIM, vel_pub, true, true);
        stepDistance(0.05, SPEED_LIM, vel_pub, true);
        stepDistance(0.05, SPEED_LIM, vel_pub, true, true);

    }
    else if (emotion == 5){ // Surprise

        if (verbose){
            ROS_INFO("Victim seems to be surprised so the robot is also surprised");
        }

        stepDistance(0.05, SPEED_LIM, vel_pub, true, true);
        sc.playWave(path_to_sounds + "surprise.wav"); // play Sound

        std::string current_video = "surprised_surprised.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);
        stepDistance(0.05, SPEED_LIM, vel_pub, true);

    }
    else if (emotion == 6){ // Neutral

        if (verbose){
            ROS_INFO("Victim seems to be neutral so the robot is discontent that they are not taking it seriously");
        }

        sc.playWave(path_to_sounds + "neutral.wav"); // play Sound

        std::string current_video = "neutral_discontent.mp4";
        PlayVideo(path_to_videos, current_video, video_window_name);

        rotByAngle(-M_PI/12, vel_pub);
        rotByAngle(M_PI/6, vel_pub);
        rotByAngle(-M_PI/12, vel_pub);  

        
    }

    else {
        if (verbose){
            ROS_INFO("Invalid emotion type");
        }
    }
}

