#ifndef __VIDEO_H_INCLUDED__
#define __VIDEO_H_INCLUDED__

#include "contest3.h"

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

extern void PlayVideo(std::string video_path, std::string video_name, std::string window_name);

#endif