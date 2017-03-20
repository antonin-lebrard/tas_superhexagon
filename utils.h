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

struct WorkPoint {
    Point2i hitPoint;
    double angleNow;
    double angleBefore;
};


class Utils {
public:
    static void launchRaycasting(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay);
    static void filterOutNotWall(Mat& grayscale);
    static vector<TriangleDetected> computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay);
    static void detectTriangle(Mat& img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected);
private:
    static Point2d directionMiddleToTriangle(const TriangleDetected& triangle, const Point2i& middle);
    static Point2d rotateVectorByAngle(const Point2d& vec, const Point2i& pivot, const double angle);
    static void removeTriangle(const TriangleDetected &triangle, Mat &grayscale);
    static bool isOnBorder(const Point2i& p, const Mat& img);
    static Point2i direction(const Point2i& p1, const Point2i& p2);
    static double angle(const Point2i& p1, const Point2i& p2);
    static WorkPoint constructWork(Mat& img, const RaycastHit& hit1, const RaycastHit& hit2, const Point2i& middle);
    static vector<vector<RaycastHit>> createZones(Mat& img, const vector<RaycastHit>& collisions, const vector<WorkPoint>& workP);
    static void drawZones(Mat& img, const vector<vector<RaycastHit>>& zones, const Point2i& middle);
};


#endif //PROJECT_UTILS_H
