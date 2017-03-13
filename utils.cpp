//
// Created by user on 3/6/17.
//

#include "utils.h"

void Utils::filterOutNotWall(Mat& grayscale) {
    uchar subDelete = 50, upDelete = 255 ;
    for (MatIterator_<uchar> it = grayscale.begin<uchar>(); it != grayscale.end<uchar>(); it++) {
        if (*it < subDelete || *it > upDelete) {
            *it = (uchar) 0;
        }
        else {
            *it = (uchar) 255;
        }
    }
    /// concentrate into the middle of the image
    for (int i = 0; i < grayscale.rows; i++){
        for (int j = 0; j < grayscale.cols; j++){
            if (i < 50 || j < 150 || j > grayscale.cols - 210)
                grayscale.at<uchar>(i, j) = 0;
        }
    }
}

void Utils::detectTriangle(Mat &img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected) {
    vector<Point> approxTriangle;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.1, true);
        if(approxTriangle.size() == 3){
            TriangleDetected triangleDetected;
            triangleDetected.points = approxTriangle;
            for (int j = 0; j < 3; j++){
                if (j > 0)
                    triangleDetected.perimeterSquared += abs(approxTriangle[j].x - approxTriangle[j-1].x)
                                                         + abs(approxTriangle[j].y - approxTriangle[j-1].y);
            }
            triangleDetected.perimeterSquared += abs(approxTriangle[2].x - approxTriangle[0].x)
                                                 + abs(approxTriangle[2].y - approxTriangle[0].y);
            //cout << "x: " << approxTriangle[0].x << " y: " << approxTriangle[0].y << endl;
            if (approxTriangle[0].x > 290 && approxTriangle[0].x < 460 && approxTriangle[0].y > 150 && approxTriangle[0].y < 330) {
                if (triangleDetected.perimeterSquared < 50.0) {
                    for (int j = 0; j < 3; j++) {
                        circle(img, approxTriangle[j], 2, Scalar(0, 0, 255), -1);
                    }
                    trianglesDetected.push_back(triangleDetected);
                }
            }
        }
    }
    if (trianglesDetected.size() == 0)
        Globals::stopAtThisFrame = true;
}

vector<TriangleDetected> Utils::computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay) {
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( grayscale, threshold_output, Globals::thresh, 255, THRESH_BINARY );

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    drawingContours = Mat::zeros(threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ) {
        Scalar color = Scalar( 255, 255, 255 );
        drawContours( drawingContours, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
    vector<TriangleDetected> trianglesDetected;
    Utils::detectTriangle(drawingContours, contours, trianglesDetected);
    return trianglesDetected;
}

void Utils::launchRaycasting(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay) {
    Point2i middle = Point2i(grayscale.cols / 2, grayscale.rows / 2);
    Utils::filterOutNotWall(grayscale);
    vector<TriangleDetected> triangles = Utils::computeContours(grayscale, drawingContours, videoDisplay);
    vector<RaycastHit> raycastCollisions = vector<RaycastHit>();
    if (triangles.size() > 0){
        Point2d directionToTriangle = directionMiddleToTriangle(triangles[0], middle);
        dilateTriangleToRect(triangles[0], grayscale);
        Scalar colorRaycast = Scalar(0,0,255);
        raycastCollisions.push_back(
                Raycast::detectCollision(
                        grayscale, drawingContours, directionToTriangle, 4, colorRaycast
                )
        );
        const double angleStep = CV_PI / 180.0 * 22.5;
        bool isStepMinus = false;
        double currentAngle;
        for (int i = 0; i < 15; i++){
            double step = i / 2 + 1;
            currentAngle = isStepMinus ? - (angleStep * step) : (angleStep * step);
            isStepMinus = !isStepMinus;
            Point2d direction = rotateVectorByAngle(directionToTriangle, middle, currentAngle);
            raycastCollisions.push_back(Raycast::detectCollision(grayscale, drawingContours, direction, 2, colorRaycast));
        }
        // TODO : maybe possible to do it while computing RaycastHits
        int maxDistanceSquared = raycastCollisions[0].distanceSquared;
        int idx = 0;
        for (int i = 1; i < 16; i++) {
            int diff = raycastCollisions[i].distanceSquared - maxDistanceSquared;
            if (diff > 1000 && !raycastCollisions[i].invalidate){
                maxDistanceSquared = raycastCollisions[i].distanceSquared;
                idx = i;
            }
        }
        circle(drawingContours, raycastCollisions[idx].stoppingPoint, 16, Scalar(0, 255, 0), -1);
    }
}

Point2d Utils::directionMiddleToTriangle(const TriangleDetected& triangle, const Point2i& middle) {
    vector<Point2d> dirs = vector<Point2d>();
    int idxBestDir = 0;
    dirs.push_back(Point2d(triangle.points[0].x - middle.x, triangle.points[0].y - middle.y));
    double distanceSquared, maxDistanceSquared = (dirs[0].x * dirs[0].x) + (dirs[0].y * dirs[0].y);
    for (int i = 1; i < 3; i++) {
        dirs.push_back(Point2i(triangle.points[i].x - middle.x, triangle.points[i].y - middle.y));
        distanceSquared = (dirs[i].x * dirs[i].x) + (dirs[i].y * dirs[i].y);
        if (distanceSquared > maxDistanceSquared) {
            idxBestDir = i;
            maxDistanceSquared = distanceSquared;
        }
    }
    double trueDist = sqrt(maxDistanceSquared);
    dirs[idxBestDir].x /= trueDist;
    dirs[idxBestDir].y /= trueDist;
    return dirs[idxBestDir];
}

Point2d Utils::rotateVectorByAngle(const Point2d& vec, const Point2i& pivot, const double angle) {
    Point2d vecRotated = Point2d(
            vec.x * cos(angle) - vec.y * sin(angle),
            vec.x * sin(angle) + vec.y * cos(angle)
    );
    return vecRotated;
}

void Utils::dilateTriangleToRect(const TriangleDetected& triangle, Mat& grayscale) {
    int xLeftMost = triangle.points[0].x, xRightMost = triangle.points[0].x, yUpMost = triangle.points[0].y, yDownMost = triangle.points[0].y;
    for (int i = 1; i < triangle.points.size(); i++) {
        if (triangle.points[i].x < xLeftMost)
            xLeftMost = triangle.points[i].x;
        if (triangle.points[i].x > xRightMost)
            xRightMost = triangle.points[i].x;
        if (triangle.points[i].y < yUpMost)
            yUpMost = triangle.points[i].y;
        if (triangle.points[i].y > yDownMost)
            yDownMost = triangle.points[i].y;
    }
    rectangle(grayscale, Point2i(xLeftMost, yUpMost), Point2i(xRightMost, yDownMost), Scalar(255,255,255), CV_FILLED);
}

