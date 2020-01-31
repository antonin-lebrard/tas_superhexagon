//
// Created by lebrarda on 06/03/17.
//

#include "Raycast.h"

/**
 * Stop at third point not equal to 0
 * @param img grayscale image
 * @param drawing where to draw circle in
 * @param vectorLine direction of the ray
 * @param isTriangleOnTheWay if triangle is in direction of ray, so should be ignored
 * @return point of collision with first wall in direction of vectorLine
 */
RaycastHit Raycast::detectCollision(Mat& img, Mat& drawing, Mat& debug, const Point2d& vectorLine, int countChangeOfColorToIgnore, const Scalar& color, const int idxRepresentingDistFromTriangle) {
    RaycastHit hit = RaycastHit();
    const Point2i initialPoint = Point2i(img.cols / 2, img.rows / 2);
    Point2i current = Point2i(img.cols / 2, img.rows / 2);
    const double absX = abs(vectorLine.x), absY = abs(vectorLine.y);
    uchar lastColor = img.at<uchar>(initialPoint);
    int countCurrentNbPixelsWhite = 0;
    //int countChangeOfColorToIgnore = 2 + (triangleOnTheWay ? 2 : 0);
    if (absX > absY){
        if (vectorLine.x < 0.0) {
            for (int i = current.x; i >= 0; i--) {
                if (algoXStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, i))
                    break;
                if (lastColor > 100){
                    countCurrentNbPixelsWhite++;
                    // if the raycast follow the limit of a block coming on the triangle, it will serves nothing to us,
                    // as we cannot be sure of which side of the limit is passable for the triangle or not.
                    if (countCurrentNbPixelsWhite > 50)
                        hit.invalidate = true;
                } else {
                    countCurrentNbPixelsWhite = 0;
                }
                circle(drawing, current, 0, color, 0);
                circle(debug, current, 0, color, 0);
            }
        }
        else {
            for (int i = current.x; i < img.cols; i++) {
                if (algoXStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, i))
                    break;
                if (lastColor > 100){
                    countCurrentNbPixelsWhite++;
                    if (countCurrentNbPixelsWhite > 50)
                        hit.invalidate = true;
                } else {
                    countCurrentNbPixelsWhite = 0;
                }
                circle(drawing, current, 0, color, 0);
                circle(debug, current, 0, color, 0);
            }
        }
    }
    else {
        if (vectorLine.y < 0.0) {
            for (int j = current.y; j >= 0; j--){
                if (algoYStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, j))
                    break;
                if (lastColor > 100){
                    countCurrentNbPixelsWhite++;
                    if (countCurrentNbPixelsWhite > 50)
                        hit.invalidate = true;
                } else {
                    countCurrentNbPixelsWhite = 0;
                }
                circle(drawing, current, 0, color, 0);
                circle(debug, current, 0, color, 0);
            }
        }
        else {
            for (int j = current.y; j < img.rows; j++){
                if (algoYStopOnCollision(img, lastColor, countChangeOfColorToIgnore, current, vectorLine, initialPoint, j))
                    break;
                if (lastColor > 100){
                    countCurrentNbPixelsWhite++;
                    if (countCurrentNbPixelsWhite > 50)
                        hit.invalidate = true;
                } else {
                    countCurrentNbPixelsWhite = 0;
                }
                circle(drawing, current, 0, color, 0);
                circle(debug, current, 0, color, 0);
            }
        }
    }
    circle(drawing, current, 0, color, 0);
    circle(debug, current, 0, color, 0);
    hit.stoppingPoint = current;
    int x = current.x - initialPoint.x;
    int y = current.y - initialPoint.y;
    hit.distanceSquared = abs(x)+abs(y);//(x * x) + (y * y);
    hit.idxRepresentingDistFromTriangle = idxRepresentingDistFromTriangle;
//    Globals::textOnImage(drawing, hit.invalidate ? "T" : "F", hit.stoppingPoint);
    return hit;
}


bool Raycast::isPointInsideMat(const Mat &img, const Point2i &toCheck) {
    Rect r = Rect(0, 0, img.cols, img.rows);
    return toCheck.inside(r);
}


bool Raycast::hasColorChanged(const Mat& img, uchar& lastColor, const Point2i& toCheck) {
    if (!isPointInsideMat(img, toCheck))
        return true;
    uchar grayscale = img.at<uchar>(toCheck);
    if (grayscale != lastColor){
        lastColor = grayscale;
        return true;
    }
    return false;
}

void Raycast::progressPointOnLineFromXFormula(Point2i &pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int &x) {
    pointToMove.x = x;
    pointToMove.y = (int) (initialPoint.y + lineVec.y * (pointToMove.x - initialPoint.x) / lineVec.x);
}

void Raycast::progressPointOnLineFromYFormula(Point2i &pointToMove, const Point2d& lineVec, const Point2i& initialPoint, const int &y) {
    pointToMove.y = y;
    pointToMove.x = (int) (initialPoint.x + lineVec.x * (pointToMove.y - initialPoint.y) / lineVec.y);
}

bool Raycast::algoXStopOnCollision(const Mat& img, uchar& lastColor, int& nbChangeToIgnore,
                                   Point2i& pointToMove, const Point2d& lineVec,
                                   const Point2i& initialPoint, const int& x) {
    progressPointOnLineFromXFormula(pointToMove, lineVec, initialPoint, x);
    if (hasColorChanged(img, lastColor, pointToMove)){
        if (nbChangeToIgnore != 0){
            nbChangeToIgnore--;
            return false;
        }
        return true;
    }
    return false;
}

bool Raycast::algoYStopOnCollision(const Mat& img, uchar& lastColor, int& nbChangeToIgnore,
                                   Point2i& pointToMove, const Point2d& lineVec,
                                   const Point2i& initialPoint, const int& y) {
    progressPointOnLineFromYFormula(pointToMove, lineVec, initialPoint, y);
    if (hasColorChanged(img, lastColor, pointToMove)){
        if (nbChangeToIgnore != 0){
            nbChangeToIgnore--;
            return false;
        }
        return true;
    }
    return false;
}

