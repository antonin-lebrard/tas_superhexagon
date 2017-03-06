//
// Created by lebrarda on 06/03/17.
//

#ifndef PROJECT_RAYCAST_H
#define PROJECT_RAYCAST_H

#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class Raycast {
private:
    static bool hasColorChanged(Mat& img, uchar& lastColor, Point2i& toCheck);
    static void progressPointOnLineFromXFormula(Point2i& pointToMove, Point2d& lineVec, Point2i& initialPoint, int& x);
    static void progressPointOnLineFromYFormula(Point2i& pointToMove, Point2d& lineVec, Point2i& initialPoint, int& y);
    static bool algoXStopOnCollision(Mat& img, uchar& lastColor, int& nbChangeToIgnore, Point2i& pointToMove, Point2d& lineVec, Point2i& initialPoint, int& x);
    static bool algoYStopOnCollision(Mat& img, uchar& lastColor, int& nbChangeToIgnore, Point2i& pointToMove, Point2d& lineVec, Point2i& initialPoint, int& y);
    
public:
    static Point2i detectCollision(Mat& img, Mat& drawing, Point2d lineVec, bool isTriangleOnTheWay);
};


#endif //PROJECT_RAYCAST_H
