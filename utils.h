//
// Created by user on 3/6/17.
//

#ifndef PROJECT_UTILS_H
#define PROJECT_UTILS_H

#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "globals.h"

using namespace std;
using namespace cv;

struct TriangleDetected {
    vector<Point> points;
    double perimeterSquared = 0.0;
};

class Utils {
public:
    static void filterOutNotWall(Mat& grayscale);
    static void detectTriangle(Mat& img, const vector<vector<Point>> &contours, vector<TriangleDetected> &trianglesDetected);
    static void computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay);
};


#endif //PROJECT_UTILS_H
