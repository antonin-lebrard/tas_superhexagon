//
// Created by user on 3/6/17.
//

#ifndef PROJECT_GLOBALS_H
#define PROJECT_GLOBALS_H

#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "string"
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

struct RaycastHit {
    Point2i stoppingPoint;
    int distanceSquared;
    bool invalidate = false;
    int idxRepresentingDistFromTriangle;
};

struct TriangleDetected {
    vector<Point> points;
    double perimeterSquared = 0.0;
};

class Globals{
public:
    static bool drawZones;
    static bool stopAtThisFrame;
    static int thresh;
    static int countIgnore;
    static void textOnImage(Mat& img, const double& d, const Point2i& p);
    static void textOnImage(Mat& img, const string& s, const Point2i& p);
};


#endif //PROJECT_GLOBALS_H
