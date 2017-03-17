//
// Created by user on 3/6/17.
//

#ifndef PROJECT_UTILS_H
#define PROJECT_UTILS_H

#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "globals.h"
#include "Raycast.h"

using namespace std;
using namespace cv;

class Utils {
public:
    static void launchRaycasting(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay);
    static void filterOutNotWall(Mat& grayscale);
    static vector<TriangleDetected> computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay);
    static void detectTriangle(Mat& img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected);
    static bool isOnBorder(const Point2i& p, const Mat& img);
private:
    static Point2d directionMiddleToTriangle(const TriangleDetected& triangle, const Point2i& middle);
    static Point2d rotateVectorByAngle(const Point2d& vec, const Point2i& pivot, const double angle);
    static void dilateTriangleToRect(const TriangleDetected &triangle, Mat &grayscale);
};


#endif //PROJECT_UTILS_H
