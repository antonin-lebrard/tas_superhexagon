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
#include "list"

using namespace std;
using namespace cv;

struct WorkPoint {
    Point2i hitPoint;
    double angleNow;
    double angleBefore;
    double ratioXOnY;
};

struct Zone {
    vector<RaycastHit> hits;
    int moyenneDistance = 0;
    
    int computeMoyenne() {
        if (hits.empty())
            return 0;
        int total = 0;
        for (auto & hit : hits){
            total += hit.distanceSquared;
        }
        moyenneDistance = (int) (total / hits.size());
        return moyenneDistance;
    }
    
    void push_back(const RaycastHit& hit){
        hits.push_back(hit);
    }
    unsigned long size() const {
        return hits.size();
    }
    void insert(vector<RaycastHit>::iterator it, const RaycastHit& hit ){
        hits.insert(it, hit);
    }
    vector<RaycastHit>::iterator begin(){
        return hits.begin();
    }
    RaycastHit operator[](int i) {
        return hits[i];
    }
};


class Utils {
public:
    static void launchRaycasting(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay, Mat& debug);
    static void filterOutNotWall(Mat& grayscale);
    static vector<TriangleDetected> computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay, Mat& debug);
    static void detectTriangle(Mat& img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected);
private:
    static const Point2i INVALIDPOINT;

    static void drawGraduation(Mat& img);
    static Point2d directionMiddleToTriangle(const TriangleDetected& triangle, const Point2i& middle);
    static Point2d rotateVectorByAngle(const Point2d& vec, const Point2i& pivot, const double angle);
    static void removeTriangle(const TriangleDetected &triangle, Mat &grayscale);
    static bool isOnBorder(const Point2i& p, const Mat& img);
    static Point2i direction(const Point2i& p1, const Point2i& p2);
    static double angle(const Point2i& p1, const Point2i& p2);
    static Point2i continueOnBorder(const Mat& drawing, const Point2i& from, const Point2i& precedent, const Point2i& origin, const Point2i& towards);
    static bool collisionIsOnSameContour(const Mat& drawing, const Mat& debug, const Point2i& from, const Point2i& to);
    static WorkPoint constructWork(const Mat& img, const RaycastHit& hit1, const RaycastHit& hit2, const Point2i& middle);
    static vector<Zone> createZones(const Mat& img, const Mat& debug, const vector<RaycastHit>& collisions, const vector<WorkPoint>& workP);
    static Point2i compute2DPolygonCentroid(const vector<Point2i> &vertices);
    static void goTowardsCentroid(vector<Point2i>& vertices);
    static void drawZones(Mat& img, const vector<Zone>& zones, const Point2i& middle, const int& idxBest);
    static int findBestZone(const Mat& img, const vector<Zone>& zones);

    static vector<int> createOrdersForZones(vector<Zone> &zones, TriangleDetected &triangle);
    static bool doubleEqualsWithEpsilon(const double& d1, const double& d2, const double& epsilon);

};


#endif //PROJECT_UTILS_H
