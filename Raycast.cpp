//
// Created by lebrarda on 06/03/17.
//

#include "Raycast.h"

/**
 * Stop at third point not equal to 0
 * @param img grayscale image
 * @param vectorLine 
 * @return the third point
 */
Point2i Raycast::detectCollision(Mat& img, Mat& drawing, Point2d vectorLine, bool isTriangleOnTheWay) {
    Point2i initialPoint = Point2i(img.cols / 2, img.rows / 2);
    Point2i current = Point2i(img.cols / 2, img.rows / 2);
    double absX = abs(vectorLine.x), absY = abs(vectorLine.y);
    uchar lastColor = img.at<uchar>(initialPoint);
    int countChangeOfColorToIgnore = 2 + (isTriangleOnTheWay ? 1 : 0);
    if (absX > absY){
        if (vectorLine.x < 0.0) {
            for (int i = initialPoint.x; i >= 0; i--) {
                if (algoXStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, i))
                    break;
            }
        }
        else {
            for (int i = current.x; i < img.cols; i++) {
                if (algoXStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, i))
                    break;
            }
        }
    }
    else {
        if (vectorLine.y < 0.0) {
            for (int j = initialPoint.y; j >= 0; j--){
                if (algoYStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, j))
                    break;
            }
        }
        else {
            for (int j = initialPoint.y; j < img.rows; j++){
                if (algoYStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, j))
                    break;
            }
        }
    }
    circle(drawing, current, 1, Scalar(0, 0, 255), -1);
    return cv::Point2i();
}

bool Raycast::hasColorChanged(Mat& img, uchar& lastColor, Point2i& toCheck) {
    uchar grayscale = img.at<uchar>(toCheck);
    if (grayscale != lastColor){
        lastColor = grayscale;
        return true;
    }
    return false;
}

void Raycast::progressPointOnLineFromXFormula(Point2i &pointToMove, Point2d& lineVec, Point2i& initialPoint, int &x) {
    pointToMove.x = x;
    pointToMove.y = (int) (initialPoint.y + lineVec.y * (pointToMove.x - initialPoint.x) / lineVec.x);
}

void Raycast::progressPointOnLineFromYFormula(Point2i &pointToMove, Point2d& lineVec, Point2i& initialPoint, int &y) {
    pointToMove.y = y;
    pointToMove.x = (int) (initialPoint.x + lineVec.x * (pointToMove.y - initialPoint.y) / lineVec.y);
}

bool Raycast::algoXStopOnCollision(Mat& img, uchar& lastColor, int& nbChangeToIgnore, 
                                        Point2i& pointToMove, Point2d& lineVec,
                                        Point2i& initialPoint, int& x) {
    progressPointOnLineFromXFormula(pointToMove, lineVec, initialPoint, x);
    if (hasColorChanged(img, lastColor, pointToMove)){
        cout << "nbChangeToIgnore: " << nbChangeToIgnore << endl;
        if (nbChangeToIgnore != 0){
            nbChangeToIgnore--;
            return false;
        }
        return true;
    }
    return false;
}

bool Raycast::algoYStopOnCollision(Mat& img, uchar& lastColor, int& nbChangeToIgnore, 
                                        Point2i& pointToMove, Point2d& lineVec,
                                        Point2i& initialPoint, int& y) {
    progressPointOnLineFromYFormula(pointToMove, lineVec, initialPoint, y);
    if (hasColorChanged(img, lastColor, pointToMove)){
        cout << "nbChangeToIgnore: " << nbChangeToIgnore << endl;
        if (nbChangeToIgnore != 0){
            nbChangeToIgnore--;
            return false;
        }
        return true;
    }
    return false;
}
