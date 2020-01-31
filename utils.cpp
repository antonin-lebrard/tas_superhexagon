//
// Created by user on 3/6/17.
//

#include "utils.h"

const Point2i Utils::INVALIDPOINT = Point2i(-1, -1);

const Scalar green = Scalar(0, 255, 0);
void Utils::drawGraduation(Mat &img) {
    int i = 0;
    for (; i < img.cols - 30; i += 100) {
        Globals::textOnImage(img, double(i), Point(i+2, 0));
        line(img, Point(i, 0), Point(i, img.rows-1), green, 1, LINE_4, 0);
    }
    for (i = 0; i < img.rows - 10; i += 100) {
        Globals::textOnImage(img, double(i), Point(0, i-2));
        line(img, Point(0, i), Point(img.cols-1, i), green, 1, LINE_4, 0);
    }
}

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
            if (i < 50 && (j < 150 || j > grayscale.cols - 210))
                grayscale.at<uchar>(i, j) = 0;
        }
    }
}

void Utils::detectTriangle(Mat &img, const vector<vector<Point>>& contours, vector<TriangleDetected>& trianglesDetected) {
    vector<Point> approxTriangle;
    for (size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.1, true);
        if(approxTriangle.size() == 3){
            TriangleDetected triangleDetected;
            triangleDetected.points = approxTriangle;
            for (int j = 1; j < 3; j++){
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
    if (trianglesDetected.empty())
        Globals::stopAtThisFrame = true;
}

vector<TriangleDetected> Utils::computeContours(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay, Mat& debug) {
    Mat threshold_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    threshold( grayscale, threshold_output, Globals::thresh, 255, THRESH_BINARY );

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    debug = Mat::zeros(threshold_output.size(), CV_8UC3);
    drawingContours = Mat::zeros(threshold_output.size(), CV_8UC3 );
    Scalar white = Scalar(255, 255, 255);
    for( int i = 0; i < contours.size(); i++ ) {
        drawContours( debug, contours, i, white, 0, LINE_4, vector<Vec4i>(), 0, Point() );
        drawContours( drawingContours, contours, i, white, 0, LINE_4, vector<Vec4i>(), 0, Point() );
    }
    drawGraduation(debug);
    vector<TriangleDetected> trianglesDetected;
    Utils::detectTriangle(drawingContours, contours, trianglesDetected);
    return trianglesDetected;
}

void Utils::launchRaycasting(Mat& grayscale, Mat& drawingContours, Mat& videoDisplay, Mat& debug) {
    Point2i middle = Point2i(grayscale.cols / 2, grayscale.rows / 2);
    Utils::filterOutNotWall(grayscale);
    vector<TriangleDetected> triangles = Utils::computeContours(grayscale, drawingContours, videoDisplay, debug);
    vector<RaycastHit> raycastCollisions = vector<RaycastHit>();
    vector<WorkPoint> workP = vector<WorkPoint>();
    if (!triangles.empty()){
        Point2d directionToTriangle = directionMiddleToTriangle(triangles[0], middle);
        removeTriangle(triangles[0], grayscale);
        Scalar colorRaycast = Scalar(0,0,255);
        const double angleStep = CV_PI / 180.0 * 11.25;//22.5; // Gives me 32 Raycast
        double currentAngle;
        workP.push_back(WorkPoint());
        for (int i = 0; i < 32; i++) {
            currentAngle = angleStep * i;
            Point2d direction = rotateVectorByAngle(directionToTriangle, middle, currentAngle);
            // idxRepresentingDistFromTriangle go from 0 to 16, then go from 15 to 1
            raycastCollisions.push_back(Raycast::detectCollision(grayscale, drawingContours, debug, direction, 2, colorRaycast, (i < 17) ? i : 32 - i));
            if (i != 0)
                workP.push_back(constructWork(drawingContours, raycastCollisions[i], raycastCollisions[i-1], middle));
        }
        workP[0] = constructWork(drawingContours, raycastCollisions[0], raycastCollisions[31], middle);
        
        vector<Zone> zones = createZones(drawingContours, debug, raycastCollisions, workP);
        
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

const Vec3b white = Vec3b(255,255,255);
const Vec3b red = Vec3b(0,0,255);

Point2i Utils::continueOnBorder(const Mat& drawing, const Point2i& from, const Point2i& precedent, const Point2i& origin, const Point2i& towards) {
    if (isOnBorder(from, drawing)) return Utils::INVALIDPOINT;
    vector<Point2i> possiblesPoints;
    Point2i diff = from - precedent;
    possiblesPoints.push_back(from + diff);
    if (diff.x != 0 && diff.y != 0) {
        possiblesPoints.push_back(Point2i(from.x + diff.x, from.y));
        possiblesPoints.push_back(Point2i(from.x, from.y + diff.y));
        possiblesPoints.push_back(Point2i(from.x - diff.x, from.y + diff.y));
        possiblesPoints.push_back(Point2i(from.x + diff.x, from.y - diff.y));
    } else if (diff.x == 0) {
        possiblesPoints.push_back(Point2i(from.x + 1, from.y + diff.y));
        possiblesPoints.push_back(Point2i(from.x - 1, from.y + diff.y));
        possiblesPoints.push_back(Point2i(from.x + 1, from.y));
        possiblesPoints.push_back(Point2i(from.x - 1, from.y));
    } else if (diff.y == 0 ) {
        possiblesPoints.push_back(Point2i(from.x + diff.x, from.y + 1));
        possiblesPoints.push_back(Point2i(from.x + diff.x, from.y - 1));
        possiblesPoints.push_back(Point2i(from.x, from.y + 1));
        possiblesPoints.push_back(Point2i(from.x, from.y - 1));
    }
    Point2i nextWhitePoint = Utils::INVALIDPOINT;
    int bestMinDistance = INT32_MAX;
    int tmp;
    for (const auto& point : possiblesPoints) {
        if ((abs(point.x - origin.x) > 2 || abs(point.y - origin.y) > 2) && drawing.at<Vec3b>(point) == red) {
            return point;
        }
        tmp = ((point.x - towards.x) * (point.x - towards.x)) + ((point.y - towards.y) * (point.y - towards.y));
        if (drawing.at<Vec3b>(point) == white && tmp < bestMinDistance) {
            nextWhitePoint = point;
            bestMinDistance = tmp;
        }
    }
    return nextWhitePoint;
}

Point2i constructPrecedentFromTo(const Point2i& from, const Point2i& to) {
    if (to.x > from.x) {
        if (to.y > from.y)
            return Point2i(from.x - 1, from.y - 1);
        else
            return Point2i(from.x - 1, from.y + 1);
    } else {
        if (to.y > from.y)
            return Point2i(from.x + 1, from.y - 1);
        else
            return Point2i(from.x + 1, from.y + 1);
    }
}

bool Utils::collisionIsOnSameContour(const Mat& drawing, const Mat& debug, const Point2i& from, const Point2i& to) {
    if (isOnBorder(from, drawing) || isOnBorder(to, drawing)) return false;
    cout << "from " << from << " to " << to << std::endl;
    Point2i tmp;
    Point2i precedent = constructPrecedentFromTo(from, to);
    Point2i current = continueOnBorder(drawing, from, precedent, from, to);
    precedent = from;
    cout << "current " << current << " precedent " << precedent << std::endl;
    Vec3b color;
    for (int i = 2; i < 80; i++) {
        if (current == Utils::INVALIDPOINT) return false;
        tmp = current;
        current = continueOnBorder(drawing, current, precedent, from, to);
//        circle(drawing, precedent, 0, Scalar(0, 255, 0), 1);
        precedent = tmp;
        color = drawing.at<Vec3b>(current);
        cout << "current " << current << " precedent " << precedent << " color " << int(color[0]) << "," << int(color[1]) << "," << int(color[2]) << std::endl;
        if (abs(current.x - to.x) < 3 && abs(current.y - to.y) < 3 && color == red) return true;
    }
    return false;
}

WorkPoint Utils::constructWork(const Mat& img, const RaycastHit& hit1, const RaycastHit& hit2, const Point2i& middle) {
    WorkPoint work;
    work.hitPoint = hit1.stoppingPoint;
    Point2i dir = Utils::direction(hit1.stoppingPoint, hit2.stoppingPoint);
    Point2i dirBeforeToMiddle = Utils::direction(hit2.stoppingPoint, middle);
    Point2i dirToMiddle = Utils::direction(hit1.stoppingPoint, middle);
    work.angleNow = angle(dir, dirToMiddle);
    work.angleBefore = angle(dir, dirBeforeToMiddle);
    work.ratioXOnY = dir.y == 0 ? 0 : double(dir.x) / double(dir.y);
    return work;
}

vector<Zone> Utils::createZones(const Mat& img, const Mat& debug, const vector<RaycastHit>& collisions, const vector<WorkPoint>& workP) {
    //imshow("Hull Demo Next frame", img);
    //imshow("Debug Next frame", debug);
    //waitKey(1);
    vector<Zone> zones = vector<Zone>();
    zones.push_back(Zone());
    zones[0].push_back(collisions[0]);

//    double lower_bound = 59.0, upper_bound = 110.0;
    
    for (int i = 1; i < collisions.size(); i++) {
        if (collisions[i].invalidate) {
            if (zones.back().size() != 0)
                zones.push_back(Zone());
            continue;
        }
        if (isOnBorder(collisions[i].stoppingPoint, img) && isOnBorder(collisions[i-1].stoppingPoint, img)) {
            zones.back().push_back(collisions[i]);
            continue;
        }
        if ( collisionIsOnSameContour(img, debug, collisions[i].stoppingPoint, collisions[i-1].stoppingPoint)) {
//            cout << collisions[i].distanceSquared << " " << collisions[i-1].distanceSquared << std::endl;
            zones.back().push_back(collisions[i]);
            continue;
        }
        /*if (workP[i].angleNow > lower_bound && workP[i].angleNow < upper_bound) {
            cout << "added one collision to a zone through angles " << i << " " << i-1 << endl;
            zones.back().push_back(collisions[i]);
            continue;
        }*/
        zones.push_back(Zone());
        zones.back().push_back(collisions[i]);
    }
    // if last collision is not valid, return zones directly
    if (collisions[collisions.size()-1].invalidate)
        return zones;
    if (zones.size() == 1) {
        return zones;
    }

    // determine if collision.back() is to be connected to collision[0]
    bool linkLastWithFirstCollision = false;
    if (isOnBorder(collisions[0].stoppingPoint, img) && isOnBorder(collisions[collisions.size()-1].stoppingPoint, img)) {
        linkLastWithFirstCollision = true;
    } else if (collisionIsOnSameContour(img, debug, collisions[0].stoppingPoint, collisions.back().stoppingPoint)) {
        linkLastWithFirstCollision = true;
    } /*else if (workP[0].angleNow > lower_bound && workP[0].angleNow < upper_bound) {
        cout << "added one collision to a zone through angles " << 0 << " " << collisions.size()-1 << endl;
        linkLastWithFirstCollision = true;
    }*/
    if (linkLastWithFirstCollision) {
        for (int i = 0; i < zones[0].size(); i++) {
            zones.back().push_back(zones[0][i]);
        }
        // then for convenience, say that the last zone is in fact the first and remove the duplicate
        zones[0] = zones[zones.size()-1];
        zones.pop_back();
    }
    return zones;
}

Point2i Utils::compute2DPolygonCentroid(const vector<Point2i>& vertices)
{
    Point2d centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i = 0;
    for (i = 0; i < vertices.size()-1; ++i)
    {
        a = vertices[i].x * vertices[i+1].y - vertices[i+1].x * vertices[i].y;
        signedArea += a;
        centroid.x += (vertices[i].x + vertices[i+1].x) * a;
        centroid.y += (vertices[i].y + vertices[i+1].y) * a;
    }

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return Point2i(int(centroid.x), int(centroid.y));
}

void Utils::goTowardsCentroid(vector<Point2i>& vertices) {
    Point2i centroid = compute2DPolygonCentroid(vertices);
    Point2d vectorTowardCentroid;
    for (int i = 0; i < vertices.size(); i++) {
        vectorTowardCentroid.x = double(centroid.x - vertices[i].x) / 10.0;
        vectorTowardCentroid.y = double(centroid.y - vertices[i].y) / 10.0;
        vertices[i].x += int(vectorTowardCentroid.x);
        vertices[i].y += int(vectorTowardCentroid.y);
    }
}

void Utils::drawZones(Mat& img, const vector<Zone>& zones, const Point2i& middle, const int& idxBest) {
    for (int i = 0; i < zones.size(); i++) {
        if (zones[i].size() != 0) {
            vector<Point2i> sommets = vector<Point2i>();
            sommets.push_back(middle);
            for (int j = 0; j < zones[i].size(); j++){
                sommets.push_back(zones[i].hits[j].stoppingPoint);
            }
            goTowardsCentroid(sommets);
            // blue if best, green if not
            if (Globals::drawZones) {
                fillConvexPoly(img, sommets, idxBest == i ? Scalar(117, 72, 77) : Scalar(89, 117, 72));
            }
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
        if (furthestIdxs.empty()){
            furthestIdxs.push_back(idxFurthestHit);
            order[0] = i;
        }
        for (unsigned long j = furthestIdxs.size() - 1; j > -1; j--){
            //if (idxFurthestHit )
        }
    }
    return vector<int>();
}

bool Utils::doubleEqualsWithEpsilon(const double &d1, const double &d2, const double &epsilon) {
    return d1 >= d2 - epsilon && d1 <= d2 + epsilon;
}
