//
// Created by user on 3/6/17.
//

#include "globals.h"


bool Globals::stopAtThisFrame = false;
int Globals::thresh = 100;
int Globals::countIgnore = 60;

void Globals::textOnImage(Mat &img, const double &d, const Point2i &p) {
    int flooredVal = (int) d;
    string s = to_string(flooredVal);
    int xText = p.x, yText = p.y;
    if (xText >= img.cols - 30)
        xText = img.cols - 30;
    if (xText < 0)
        xText = 0;
    if (yText >= img.rows - 10)
        yText = img.rows - 10;
    if (yText < 10)
        yText = 10;
    Point2i org = Point2i(xText, yText);
    putText(img, s, org, FONT_HERSHEY_PLAIN, 1.0, Scalar(0,255,0));
}

void Globals::textOnImage(Mat &img, const string &s, const Point2i &p) {
    int xText = p.x, yText = p.y;
    if (xText >= img.cols - 30)
        xText = img.cols - 30;
    if (xText < 0)
        xText = 0;
    if (yText >= img.rows - 10)
        yText = img.rows - 10;
    if (yText < 10)
        yText = 10;
    Point2i org = Point2i(xText, yText);
    putText(img, s, org, FONT_HERSHEY_PLAIN, 1.0, Scalar(0,255,0));
}
