#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "video.h"

using namespace cv;
using namespace std;

Mat toDisplayVideo, gray, drawingContour;

int thresh = 100;

Point2i pointsToFetch[45] = {

        Point2i(50,50),
        Point2i(100,50),
        Point2i(150,50),
        Point2i(200,50),
        Point2i(250,50),

        Point2i(50,100),
        Point2i(100,100),
        Point2i(150,100),
        Point2i(200,100),
        Point2i(250,100),

        Point2i(50,150),
        Point2i(100,150),
        Point2i(150,150),
        Point2i(200,150),
        Point2i(250,150),

        Point2i(50,200),
        Point2i(100,200),
        Point2i(150,200),
        Point2i(200,200),
        Point2i(250,200),

        Point2i(50,250),
        Point2i(100,250),
        Point2i(150,250),
        Point2i(200,250),
        Point2i(250,250),

        Point2i(50,300),
        Point2i(100,300),
        Point2i(150,300),
        Point2i(200,300),
        Point2i(250,300),

        Point2i(50,350),
        Point2i(100,350),
        Point2i(150,350),
        Point2i(200,350),
        Point2i(250,350),

        Point2i(50,400),
        Point2i(100,400),
        Point2i(150,400),
        Point2i(200,400),
        Point2i(250,400),

};

void draw() {
    imshow("scene", toDisplayVideo);
    imshow("Hull demo", drawingContour);
    /// callback mouse if necessary
}

void trianglesDetect(Mat& img, const vector< vector<Point> >& contours){
    vector<Point> approxTriangle;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
        if(approxTriangle.size() == 3){
            //drawContours(img, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
            vector<Point>::iterator vertex;
            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
                circle(img, *vertex, 3, Scalar(0, 0, 255), 1);
            }
        }
    }
}

void thresh_callback(int, void* )
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    threshold( gray, threshold_output, thresh, 255, THRESH_BINARY );

    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the convex hull object for each contour
    vector<vector<Point>> hull( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        convexHull( Mat(contours[i]), hull[i], false );
    }

    /// Draw contours + hull results
    drawingContour = Mat::zeros(threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( theRNG().uniform(0, 255), theRNG().uniform(0, 255), theRNG().uniform(0, 255));
        drawContours( drawingContour, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //drawContours( drawingContour, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
    trianglesDetect(toDisplayVideo, contours);
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
    /// the zone is about 50 < x  
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
    thresh_callback( 0, 0 );
}

void initThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    namedWindow("scene", CV_WINDOW_AUTOSIZE);
    namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
    createTrackbar(" Threshold:", "Source", &thresh, 255, thresh_callback);
    thresh_callback( 0, 0 );
}

int main() {
    Video v = Video(initThings, doThings, draw);
    v.openvideo("../good30fps.flv", toDisplayVideo);
}
