#include "bumper.h"

bool bumpersPressed(){
    //return 1 if any bumpers are currently being pressed

    bool any_bumper_pressed = false;

    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= ( bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED );
    }

    return any_bumper_pressed;
}

uint8_t specificBumperPressed() {
    /* This will the specific bumper that is depressed. If there are no bumpers depressed, a failure message 
    */

    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        if (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED) {
            return b_idx;
        }
    }

    return NO_BUMPER_DEPRESSED;

}
