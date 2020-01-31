//
// Created by lebrarda on 06/03/17.
//

#ifndef PROJECT_RAYCAST_H
#define PROJECT_RAYCAST_H

#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "globals.h"

using namespace std;
using namespace cv;

//typedef bool (*AlgoStopOnCollision)(const Mat& img, uchar& lastColor, int& nbChangeToIgnore, Point2i& pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int& x);

class Raycast {
private:
    //static AlgoStopOnCollision determineWhichAlgoToCall(const double& absX, const double& absY, const Point2d& vectorLine);
    static bool isPointInsideMat(const Mat& img, const Point2i& toCheck);
    static bool hasColorChanged(const Mat& img, uchar& lastColor, const Point2i& toCheck);
    static void progressPointOnLineFromXFormula(Point2i& pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int& x);
    static void progressPointOnLineFromYFormula(Point2i& pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int& y);
    static bool algoXStopOnCollision(const Mat& img, uchar& lastColor, int& nbChangeToIgnore, Point2i& pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int& x);
    static bool algoYStopOnCollision(const Mat& img, uchar& lastColor, int& nbChangeToIgnore, Point2i& pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int& y);
    
public:
    static RaycastHit detectCollision(Mat& img, Mat& drawing, Mat& debug, const Point2d& lineVec, int countChangeOfColorToIgnore, const Scalar& color, const int idxRepresentingDistFromTriangle);
};


#endif //PROJECT_RAYCAST_H
