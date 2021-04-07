#ifndef __INTERACTION_H_INCLUDED__
#define __INTERACTION_H_INCLUDED__

#include "contest3.h"

extern void Interact(int emotion, ros::Publisher *vel_pub, std::string path_to_sounds, bool verbose = false);

#endif