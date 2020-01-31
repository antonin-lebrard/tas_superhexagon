#include <iostream>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "video.h"
#include "Raycast.h"
#include "utils.h"

using namespace cv;
using namespace std;

Mat toDisplayVideo, gray, drawingContour, drawingDebug;

void draw() {
//    imshow("scene", toDisplayVideo);
//    imshow("gray", gray);
    imshow("Hull demo", drawingContour);
    imshow("Debug", drawingDebug);
    /// callback mouse if necessary
}

void doThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    Utils::launchRaycasting(gray, drawingContour, toDisplayVideo, drawingDebug);
}

void initThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
//    namedWindow("scene", CV_WINDOW_AUTOSIZE);
//    namedWindow("gray", CV_WINDOW_AUTOSIZE);
    namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
    namedWindow("Debug", CV_WINDOW_AUTOSIZE);
    Utils::computeContours(gray, drawingContour, toDisplayVideo, drawingDebug);
}

bool stopAtFrame(){
    bool toReturn = Globals::stopAtThisFrame;
    if (Globals::countIgnore != 0){
        Globals::countIgnore--;
        Globals::stopAtThisFrame = false;
        return false;
    }
    if (Globals::stopAtThisFrame)
        Globals::stopAtThisFrame = false;
    return toReturn;
}

int main() {
    //cout << getBuildInformation() << std::endl;
    Video v = Video(initThings, doThings, draw, stopAtFrame);
    v.openVideo("/home/antonin/Dev/personel/tas_superhexagon/good30fps.flv", toDisplayVideo);
}

