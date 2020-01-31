//
// Created by user on 2/25/17.
//

#include "video.h"
#include "globals.h"

void Video::openVideo(const String& videoName, Mat &frame) {
    VideoCapture cap = VideoCapture(videoName, CAP_FFMPEG);
    // check if we succeeded
    if(!cap.isOpened()) {
        cout << "Capture did not succeed to open " << videoName << std::endl;
        return;
    }
    cap >> frame;
    this->initCallback();
    while(cap.isOpened()){
        clock_t tStart = clock();
        this->doThingsCallback();
        this->drawCallback();
        bool signalFromProg = this->stopAtFrame();
        double timeTaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
        printf("Time taken: %.2f ms\n", timeTaken*100.0);
        cap >> frame;
        char key = (char) waitKey(frameTime - (int)(timeTaken * 100.0 + 1));
        if (signalFromProg || advanceOnlyOneFrame) {
            key = 'p';
            advanceOnlyOneFrame = false;
        }
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
                    if (k == 'n') {
                        paused = false;
                        advanceOnlyOneFrame = true;
                    }
                    if (k == 'z')
                        Globals::drawZones = !Globals::drawZones;
                }
                if (stopping)
                    return;
                break;
            case 'k':
                return;
            case 'z':
                Globals::drawZones = !Globals::drawZones;
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
