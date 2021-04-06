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

    // Class to handle sounds.
    sound_play::SoundClient sc;
    //

    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    sc.playWave(path_to_sounds + "sound.wav");
    //

    stepDistance(1, SPEED_LIM, vel_pub, true);
    rotByAngle(-M_PI/6, vel_pub);


}

