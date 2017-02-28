//
// Created by user on 2/25/17.
//

#ifndef PROJECT_VIDEO_H
#define PROJECT_VIDEO_H


#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

typedef void (*Callback)();

class Video {

private:
    bool paused;
    bool stopping;
    int frameTime = 33;
    Callback initCallback;
    Callback doThingsCallback;
    Callback drawCallback;

public:
    Video(Callback initCallback, Callback doThingsCallback, Callback drawCallback);

    void openvideo(String videoName, Mat &frame);
};


#endif //PROJECT_VIDEO_H
