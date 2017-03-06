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
                        circle(img, approxTriangle[j], 3, Scalar(0, 0, 255), 1);
                    }
                    trianglesDetected.push_back(triangleDetected);
                }
            }
        }
    }
    if (trianglesDetected.size() == 0)
        Globals::stopAtThisFrame = true;
}

void Utils::computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay) {
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( grayscale, threshold_output, Globals::thresh, 255, THRESH_BINARY );

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point>> hull( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        convexHull( Mat(contours[i]), hull[i], false );
    }

    drawingContours = Mat::zeros(threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( theRNG().uniform(0, 255), theRNG().uniform(0, 255), theRNG().uniform(0, 255));
        drawContours( drawingContours, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //drawContours( drawingContour, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
    vector<TriangleDetected> trianglesDetected;
    Utils::detectTriangle(videoDisplay, contours, trianglesDetected);
}
