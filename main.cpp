#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "video.h"
#include "Raycast.h"

using namespace cv;
using namespace std;

struct TriangleDetected {
    vector<Point> points;
    double perimeterSquared = 0.0;
};

Mat toDisplayVideo, gray, drawingContour;

bool stopAtThisFrame = false;
int thresh = 100, countIgnore = 60;

void draw() {
    imshow("scene", toDisplayVideo);
    imshow("Hull demo", drawingContour);
    //imshow("gray", gray);
    /// callback mouse if necessary
}

void trianglesDetect(Mat& img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected){
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
        stopAtThisFrame = true;
}

void thresh_callback()
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( gray, threshold_output, thresh, 255, THRESH_BINARY );

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point>> hull( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        convexHull( Mat(contours[i]), hull[i], false );
    }

    drawingContour = Mat::zeros(threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( theRNG().uniform(0, 255), theRNG().uniform(0, 255), theRNG().uniform(0, 255));
        drawContours( drawingContour, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //drawContours( drawingContour, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
    vector<TriangleDetected> trianglesDetected;
    trianglesDetect(toDisplayVideo, contours, trianglesDetected);
    
}

void filterOutNotWall() {
    uchar subDelete = 50, upDelete = 255 ;
    for (MatIterator_<uchar> it = gray.begin<uchar>(); it != gray.end<uchar>(); it++) {
        if (*it < subDelete || *it > upDelete) {
            *it = (uchar) 0;
        }
        else {
            *it = (uchar) 255;
        }
    }
    /// concentrate into the middle of the image
    for (int i = 0; i < gray.rows; i++){
        for (int j = 0; j < gray.cols; j++){
            if (i < 50 || j < 150 || j > gray.cols - 210)
                gray.at<uchar>(i, j) = 0;
        }
    }
}

void doThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    filterOutNotWall();
    thresh_callback();
    Raycast::detectCollision(gray, drawingContour, Point2d(-1.0, 0.0), false);
    Raycast::detectCollision(gray, drawingContour, Point2d(0.0, -1.0), false);
    Raycast::detectCollision(gray, drawingContour, Point2d(1.0, 0.0), false);
    Raycast::detectCollision(gray, drawingContour, Point2d(0.0, 1.0), false);
}

void initThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    namedWindow("scene", CV_WINDOW_AUTOSIZE);
    namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
    thresh_callback();
}

bool stopAtFrame(){
    bool toReturn = stopAtThisFrame;
    if (countIgnore != 0){
        countIgnore--;
        stopAtThisFrame = false;
        return false;
    }
    if (stopAtThisFrame)
        stopAtThisFrame = false;
    return toReturn;
}

int main() {
    Video v = Video(initThings, doThings, draw, stopAtFrame);
    v.openvideo("../good30fps.flv", toDisplayVideo);
}


