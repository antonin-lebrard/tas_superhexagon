//
// Created by user on 2/25/17.
//

#include "video.h"

void Video::openvideo(String videoName, Mat &frame) {
    VideoCapture cap;
    cap.open(videoName);
    if(!cap.isOpened())  // check if we succeeded
        return;
    cap >> frame;
    this->initCallback();
    while(cap.isOpened()){
        clock_t tStart = clock();
        this->doThingsCallback();
        this->drawCallback();
        bool signalFromProg = this->stopAtFrame();
        double timeTaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
        //printf("Time taken: %.2fs\n", timeTaken*100.0);
        cap >> frame;
        char key = (char) waitKey(frameTime - (int)(timeTaken * 100.0 + 1));
        if (signalFromProg)
            key = 'p';
        switch (key){
            case 'p':
                paused = true;
                while(paused){
                    char k = (char) waitKey(frameTime);
                    if (k == 'p')
                        paused = false;
                    if (k == 'k') {
                        stopping = true;
                        paused = false;
                    }
                }
                if (stopping)
                    return;
                break;
            case 'k':
                return;
            default:
                break;
        }
    }
}

Video::Video(Callback initCallback, Callback doThingsCallback, Callback drawCallback, IsWaitingCallback stopAtFrame)
{
    this->initCallback = initCallback;
    this->doThingsCallback = doThingsCallback;
    this->drawCallback = drawCallback;
    this->stopAtFrame = stopAtFrame;
}
