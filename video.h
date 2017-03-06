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
typedef bool (*IsWaitingCallback)();

class Video {

private:
    bool paused = false;
    bool stopping = false;
    int frameTime = 33;
    Callback initCallback;
    Callback doThingsCallback;
    Callback drawCallback;
    IsWaitingCallback stopAtFrame;

public:
    Video(Callback initCallback, Callback doThingsCallback, Callback drawCallback, IsWaitingCallback stopAtFrame);

    void openvideo(String videoName, Mat &frame);
};


#endif //PROJECT_VIDEO_H
