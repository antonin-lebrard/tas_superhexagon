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
                    /*for (int j = 0; j < 3; j++) {
                        circle(img, approxTriangle[j], 2, Scalar(0, 0, 255), -1);
                    }*/
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
        const double angleStep = CV_PI / 180.0 * 11.25;//22.5;
        double currentAngle;
        workP.push_back(WorkPoint());
        for (int i = 0; i < 32; i++) {
            currentAngle = angleStep * i;
            Point2d direction = rotateVectorByAngle(directionToTriangle, middle, currentAngle);
            // idxRepresentingDistFromTriangle go from 0 to 16, then go from 15 to 1
            raycastCollisions.push_back(Raycast::detectCollision(grayscale, drawingContours, direction, 2, colorRaycast, (i < 17) ? i : 32 - i));
            if (i != 0)
                workP.push_back(constructWork(drawingContours, raycastCollisions[i], raycastCollisions[i-1], middle));
        }
        workP[0] = constructWork(drawingContours, raycastCollisions[0], raycastCollisions[15], middle);
        
        vector<Zone> zones = createZones(drawingContours, raycastCollisions, workP);
        
        vector<int> order = createOrdersForZones(zones, triangles[0]);
        
        int idx = findBestZone(grayscale, zones);
        
        drawZones(drawingContours, zones, middle, idx);
        
        // redraw triangle
        for (int i = 0; i < triangles[0].points.size(); i++){
            circle(drawingContours, triangles[0].points[i], 2, Scalar(0, 0, 255), -1);
        }
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

WorkPoint Utils::constructWork(const Mat& img, const RaycastHit& hit1, const RaycastHit& hit2, const Point2i& middle) {
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

vector<Zone> Utils::createZones(const Mat& img, const vector<RaycastHit>& collisions, const vector<WorkPoint>& workP) {
    vector<Zone> zones = vector<Zone>();
    zones.push_back(Zone());
    zones[0].push_back(collisions[0]);

    double lower_bound = 59.0, upper_bound = 110.0;
    
    for (int i = 1; i < collisions.size() - 1; i++) {
        if (collisions[i].invalidate) {
            if (zones[zones.size()-1].size() != 0)
                zones.push_back(Zone());
            continue;
        }
        if (isOnBorder(collisions[i].stoppingPoint, img)) {
            if (isOnBorder(collisions[i-1].stoppingPoint, img)) {
                zones[zones.size() - 1].push_back(collisions[i]);
                continue;
            }
        }
        if (workP[i].angleNow < lower_bound || workP[i].angleNow > upper_bound) {
            zones.push_back(Zone());
            zones[zones.size()-1].push_back(collisions[i]);
            continue;
        }
        zones[zones.size()-1].push_back(collisions[i]);
    }
    // if last collision is not valid, return zones directly
    if (collisions[collisions.size()-1].invalidate)
        return zones;
    
    // last one
    // last collision touch border, first too, should link them
    bool linkLastWithFirstCollision = false;
    if (isOnBorder(collisions[0].stoppingPoint, img) && isOnBorder(collisions[collisions.size()-1].stoppingPoint, img)) {
        linkLastWithFirstCollision = true;
    }
    // first seems to be in same zone as last, should link them
    else if (workP[0].angleNow > lower_bound || workP[0].angleNow < upper_bound) {
        linkLastWithFirstCollision = true;
    }
    if (linkLastWithFirstCollision) {
        // last seems to not be connect with last - 1 collision
        if (workP[workP.size()-1].angleNow < lower_bound || workP[workP.size()-1].angleNow > upper_bound 
                    && !(isOnBorder(collisions[collisions.size()-1].stoppingPoint, img) && isOnBorder(collisions[collisions.size()-2].stoppingPoint, img))){
            zones[0].insert(zones[0].begin(), collisions[collisions.size()-1]);
        }
        // last seems to be connected with last - 1 collision
        else {
            if (zones.size() == 1) {
                zones[0].push_back(collisions[collisions.size()-1]);
                return zones;
            }
            zones[zones.size()-1].push_back(collisions[collisions.size()-1]);
            // take all raycast from first zone and push them to last zone
            for (int i = 0; i < zones[0].size(); i++){
                zones[zones.size()-1].push_back(zones[0][i]);
            }
            // then for convenience, say that the last zone is in fact the first and remove the duplicate
            zones[0] = zones[zones.size()-1];
            zones.pop_back();
        }
    } else {
        zones.push_back(Zone());
        zones[zones.size()-1].push_back(collisions[collisions.size()-1]);
    }
    return zones;
}

void Utils::drawZones(Mat& img, const vector<Zone>& zones, const Point2i& middle, const int& idxBest) {
    for (int i = 0; i < zones.size(); i++) {
        if (zones[i].size() != 0) {
            vector<Point2i> sommets = vector<Point2i>();
            sommets.push_back(middle);
            for (int j = 0; j < zones[i].size(); j++){
                sommets.push_back(zones[i].hits[j].stoppingPoint);
            }
            // blue if best, green if not
            fillConvexPoly(img, sommets, idxBest == i ? Scalar(255,0,0) : Scalar(0,255,0));
        }
    }
}

int Utils::findBestZone(const Mat& img, const vector<Zone>& zones) {
    int max = 0, tmp = 0, idx = -1;
    for (int i = 0; i < zones.size(); i++) {
        tmp = zones[i].moyenneDistance;
        if (tmp > max) {
            idx = i;
            max = tmp;
        }
    }
    return idx;
}

// TODO : finish and change into LinkedList (I'm dumb)
vector<int> Utils::createOrdersForZones(vector<Zone> &zones, TriangleDetected &triangle) {
    vector<int> order = vector<int>(zones.size());
    vector<int> furthestIdxs;
    for (int i = 0; i < zones.size(); i++){
        zones[i].computeMoyenne();
        int idxFurthestHit = 0;
        for (int j = 0; j < zones[i].size(); j++){
            if (zones[i][j].idxRepresentingDistFromTriangle == 0){
                idxFurthestHit = 0;
                break;
            }
            if (idxFurthestHit < zones[i][j].idxRepresentingDistFromTriangle)
                idxFurthestHit = zones[i][j].idxRepresentingDistFromTriangle;
        }
        if (furthestIdxs.size() == 0){
            furthestIdxs.push_back(idxFurthestHit);
            order[0] = i;
        }
        for (unsigned long j = furthestIdxs.size() - 1; j > -1; j--){
            //if (idxFurthestHit )
        }
    }
    return vector<int>();
}

