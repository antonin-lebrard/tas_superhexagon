#include <iostream>
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

Mat toDisplayVideo, colorImg, gray, drawingContour;

ORB orb = ORB(150, 1.5, 8 /*, 2, 0, 2, ORB::HARRIS_SCORE, 2*/);

vector<KeyPoint> kp;

vector<Point2f> kpPoints;
vector<char> isKpGood;

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
    //imshow("gray", gray);
    //imshow("brighter color", colorImg);
    imshow("Hull demo", drawingContour);

    //imshow("filter", filterImg);
    /// callback mouse if necessary
}

/*void detectAndDescribe(){
    orb.detect(toDisplayVideo, kp);
    //orb.compute(toDisplayVideo, kp, dp);
    KeyPoint::convert(kp, kpPoints);
    isKpGood.assign(kpPoints.size(), 1);
}

void filterPoints(){
    filterImg = Mat::zeros(filterImg.size(), filterImg.type());
    for (int i = 0; i < kpPoints.size(); i++) {
        if (filterImg.at<uchar>(kpPoints[i]) != 0){
            isKpGood[i] = 0;
        } else {
            circle(filterImg, kpPoints[i], 7, Scalar(255,255,255), -1);
        }
    }
}

void displayKeyPoints(){
    for (int i = 0; i < kpPoints.size(); i++) {
        if (isKpGood[i] == 1)
            circle(toDisplayVideo, kpPoints[i], 2, Scalar(0,0,255), -1);
    }
}*/

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
        Scalar color = Scalar( 255, 0, 0 );
        drawContours( drawingContour, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        drawContours( drawingContour, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
}

void detectWallColor(){
    /*vector<Scalar> colors;
    for (Point2i p : pointsToFetch){
        Vec3b data = toDisplayVideo.at<Vec3b>(p);
        Scalar col(data[0], data[1], data[2]);
        circle(toDisplayVideo, p, 18, col, -1);
        if (colors.empty())
            colors.push_back(col);
        for (Scalar s : colors){
            if ( abs(s[0] - col[0]) > 2 &&
                 abs(s[1] - col[1]) > 2 &&
                 abs(s[2] - col[2]) > 2)
                colors.push_back(col);
        }
    }
    rectangle(colorImg, Point2i(0,0), Point2i(50,50), Scalar(0,0,0), -1);
    if (colors.size() > 0)
        rectangle(colorImg, Point2i(0,0), Point2i(25,25), colors[0], -1);
    if (colors.size() > 1)
        rectangle(colorImg, Point2i(25,0), Point2i(50,25), colors[1], -1);
    if (colors.size() > 2)
        rectangle(colorImg, Point2i(25,25), Point2i(50,50), colors[2], -1);*/
}

void filterOutNotWall(){
    uchar subDelete = 50;
    for (MatIterator_<uchar> it = gray.begin<uchar>(); it != gray.end<uchar>(); it++){
        if (*it < subDelete){
            *it = (uchar)0;
        }
    }
}

void doThings(){
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    detectWallColor();
    filterOutNotWall();
    thresh_callback( 0, 0 );
}

void initThings(){
    colorImg = Mat::zeros(50, 50, toDisplayVideo.type());
    cvtColor(toDisplayVideo, gray, COLOR_RGB2GRAY);
    namedWindow("scene", CV_WINDOW_AUTOSIZE);
    namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
    createTrackbar(" Threshold:", "Source", &thresh, 255, thresh_callback);
    thresh_callback( 0, 0 );
}

void video(char* videoName){
    VideoCapture cap;
    if (videoName != NULL)
        cap.open(videoName);
    else
        cap = VideoCapture(0);
    if(!cap.isOpened())  // check if we succeeded
        return;
    cap >> toDisplayVideo;
    initThings();
    while(cap.isOpened()){
        clock_t tStart = clock();
        doThings();
        draw();
        double timeTaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
        //printf("Time taken: %.2fs\n", timeTaken*100.0);
        cap >> toDisplayVideo;
        if (waitKey(33 - (int)(timeTaken * 100.0 + 1)) >= 0)
            return;
    }
}

int main() {
    video("../good30fps.flv");
}

