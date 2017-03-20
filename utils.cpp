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
    vector<WorkPoint> workP = vector<WorkPoint>();
    if (triangles.size() > 0){
        Point2d directionToTriangle = directionMiddleToTriangle(triangles[0], middle);
        removeTriangle(triangles[0], grayscale);
        Scalar colorRaycast = Scalar(0,0,255);
        const double angleStep = CV_PI / 180.0 * 22.5;
        double currentAngle;
        workP.push_back(WorkPoint());
        for (int i = 0; i < 16; i++) {
            currentAngle = angleStep * i;
            Point2d direction = rotateVectorByAngle(directionToTriangle, middle, currentAngle);
            raycastCollisions.push_back(Raycast::detectCollision(grayscale, drawingContours, direction, 2, colorRaycast));
            if (i != 0)
                workP.push_back(constructWork(drawingContours, raycastCollisions[i], raycastCollisions[i-1], middle));
        }
        workP[0] = constructWork(drawingContours, raycastCollisions[0], raycastCollisions[15], middle);
        
        vector<vector<RaycastHit>> zones = createZones(drawingContours, raycastCollisions, workP);
        
        drawZones(drawingContours, zones, middle);
        
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

void Utils::removeTriangle(const TriangleDetected &triangle, Mat &grayscale) {
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
    rectangle(grayscale, Point2i(xLeftMost-1, yUpMost-1), Point2i(xRightMost+1, yDownMost+1), Scalar(0,0,0), CV_FILLED);
}

bool Utils::isOnBorder(const Point2i& p, const Mat &img) {
    int w = img.cols;
    int h = img.rows;
    if (p.x == 0 || p.x == -1 || p.x == 1)
        return true;
    if (p.x == w-1 || p.x == w || p.x == w+1)
        return true;
    if (p.y == 0 || p.y == -1 || p.y == 1)
        return true;
    if (p.y == h-1 || p.y == h || p.y == h+1)
        return true;
    return false;
}

Point2i Utils::direction(const Point2i &p1, const Point2i &p2) {
    return Point2i(p2.x - p1.x, p2.y - p1.y);
}

double Utils::angle(const Point2i &p1, const Point2i &p2) {
    int ps = p1.x * p2.x + p1.y * p2.y;
    if (ps == 0)
        return 90.0;
    double lp1 = sqrt(p1.x * p1.x + p1.y * p1.y);
    double lp2 = sqrt(p2.x * p2.x + p2.y * p2.y);
    double cosp = (ps / lp1) / lp2;
    return acos(cosp) * (180 / CV_PI);
}

WorkPoint Utils::constructWork(Mat& img, const RaycastHit& hit1, const RaycastHit& hit2, const Point2i& middle) {
    WorkPoint work;
    work.hitPoint = hit1.stoppingPoint;
    Point2i dir = Utils::direction(hit1.stoppingPoint, hit2.stoppingPoint);
    Point2i dirBeforeToMiddle = Utils::direction(hit2.stoppingPoint, middle);
    Point2i dirToMiddle = Utils::direction(hit1.stoppingPoint, middle);
    double angleNow = angle(dir, dirToMiddle);
    work.angleNow = angleNow;
    double angleBefore = angle(dir, dirBeforeToMiddle);
    work.angleBefore = angleBefore;
    /*if (!hit1.invalidate) {
        Globals::textOnImage(img, angleNow, hit1.stoppingPoint);
        Globals::textOnImage(img, angleBefore, Point2i(hit1.stoppingPoint.x - 40, hit1.stoppingPoint.y));
    }*/
    return work;
}

vector<vector<RaycastHit>> Utils::createZones(Mat& img, const vector<RaycastHit>& collisions, const vector<WorkPoint>& workP) {
    vector<vector<RaycastHit>> zones = vector<vector<RaycastHit>>();
    zones.push_back(vector<RaycastHit>());
    zones[0].push_back(collisions[0]);

    for (int i = 1; i < collisions.size() - 1; i++) {
        if (collisions[i].invalidate) {
            if (zones[zones.size()-1].size() != 0)
                zones.push_back(vector<RaycastHit>());
            continue;
        }
        if (isOnBorder(collisions[i].stoppingPoint, img) && isOnBorder(collisions[i-1].stoppingPoint, img)) {
            zones[zones.size()-1].push_back(collisions[i]);
            continue;
        }
        if (workP[i].angleNow < 50.0 || workP[i].angleNow > 120.0) {
            zones.push_back(vector<RaycastHit>());
            zones[zones.size()-1].push_back(collisions[i]);
        }
        else {
            zones[zones.size()-1].push_back(collisions[i]);
        }
    }
    if (collisions[collisions.size()-1].invalidate)
        return zones;
    /*if (isOnBorder(collisions[0].stoppingPoint, img) && isOnBorder(collisions[collisions.size()-1].stoppingPoint, img)) {
        
    }*/
    if (zones.size() == 1)
        return zones;
    // last one
    // if not in same zone as last - 1 raycast, go test with first raycast
    if (workP[workP.size()-1].angleNow < 50.0 || workP[workP.size()-1].angleNow > 120.0) {
        // seems to be same zone as first raycast
        if (workP[0].angleNow > 50.0 || workP[0].angleNow < 120.0) {
            zones[0].insert(zones[0].begin(), collisions[collisions.size()-1]);
        } else {
            zones[zones.size()-1].push_back(collisions[collisions.size()-1]);
        }
    } else {
        // push back last raycast to last zone
        zones[zones.size()-1].push_back(collisions[collisions.size()-1]);
        // if first raycast is also in last zone 
        if (workP[0].angleNow > 50.0 || workP[0].angleNow < 120.0) {
            // take all raycast from first zone and push them to last zone
            for (int i = 0; i < zones[0].size(); i++){
                zones[zones.size()-1].push_back(zones[0][i]);
            }
            // then for convenience, say that the last zone is in fact the first and remove the duplicate
            zones[0] = zones[zones.size()-1];
            zones.pop_back();
        }
    }
    return zones;
}

void Utils::drawZones(Mat& img, const vector<vector<RaycastHit>>& zones, const Point2i& middle) {
    for (int i = 0; i < zones.size(); i++) {
        if (zones[i].size() != 0) {
            vector<Point2i> sommets = vector<Point2i>();
            sommets.push_back(middle);
            for (int j = 0; j < zones[i].size(); j++){
                sommets.push_back(zones[i][j].stoppingPoint);
            }
            fillConvexPoly(img, sommets, Scalar(0,255,0));
        }
    }
}

