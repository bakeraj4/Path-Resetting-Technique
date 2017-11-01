#include <RedirectedWalking/PRT/Segment.h>
#include <exception>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#pragma once

using namespace Hive;

/**
* Default constructor that makes a line from <0,0,0,1> to <0,0,0,1>.
*/
Segment::Segment() {
    vertex1 = glm::vec4(0.0, 0.0, 0.0, 1.0);
    vertex2 = glm::vec4(0.0, 0.0, 0.0, 1.0);
    orientation = 0.0;
    length = 0.0;
}

/**
* This constructor takes two x,y pairs as end points.
* @param x1 - The x coordinate of the first vertex.
* @param y1 - The y coordinate of the first veretex.
* @param x2 - The x coordinate of the second vertex.
* @param y2 - The y coordinate of the second vertex.
*/
Segment::Segment(double x1, double y1, double x2, double y2) {
    vertex1 = glm::vec4(x1, y1, 0.0, 1.0);
    vertex2 = glm::vec4(x2, y2, 0.0, 1.0);

    orientation = Util::NormAngle(Util::R2D(-std::atan2(x2 - x1, y2 - y1)));

    length = glm::distance(vertex1, vertex2);
}

/**
* Copy constructor. This way I can control how things are being coppied.
* @param other - The segment to copy.
*/
Segment::Segment(const Segment &other) {
    vertex1 = glm::vec4(other.vertex1);
    vertex2 = glm::vec4(other.vertex2);
    orientation = other.orientation;
    length = other.length;
}

/**
* This method is used to get the length of the segment. The length is
* calculated in the constructor, so it is calculated only once.
* @return length - The length of the segment.
*/
double Segment::getLength() const {
    return length;
}

/**
* This method provides the caller with the orientation of the segment.
* @return orientation - The orientation of the segment.
*/
double Segment::getOrientation() const {
    return orientation;
}

/**
* This method is used to determine if the segment has an intersection with
* another segment and if so where. The calculation was aquired from :
* http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
* I also checked for intersections at the endpoints of the segments before the
* actual calculations. If there is an intersection the parameter intPt will
* hold the intersection point for the caller. Otherwise the glm::vec4 is filled
* with -1's.
* @param other  - The segment that is being determined if it intersects with
*                 this segment.
* @param intPt  - The container to store the intersection point.
* @return true  - Iff there is an intersection between the two segments.
* @return flase - Iff there is not an intersection between the two segments.
*/
bool Segment::hasIntersection(const Segment& other, glm::vec4 &intPt) const {
    if (getX2() == other.getX1() && getY2() == other.getY1()) {
        // segments share the second vertex of this and the first of other
        intPt = glm::vec4(getX2(), getY2(), 0.0, 1.0);
        return true;
    } else if (getX2() == other.getX2() && getY2() == other.getY2()) {
        // segments share their second vertex
        intPt = glm::vec4(getX2(), getY2(), 0.0, 1.0);
        return true;
    } else if (getX1() == other.getX1() && getY1() == other.getY1()) {
        // segments share their first vertex
        intPt = glm::vec4(getX1(), getY1(), 0.0, 1.0);
        return true;
    } else  if (getX1() == other.getX2() && getY1() == other.getY2()) {
        // segments share the first vertex of this and the second of other
        intPt = glm::vec4(getX1(), getY1(), 0.0, 1.0);
        return true;
    }

    double x1, x2, x3, x4, y1, y2, y3, y4;
    x1 = round(vertex1.x);
    x2 = round(vertex2.x);
    x3 = round(other.vertex1.x);
    x4 = round(other.vertex2.x);
    y1 = round(vertex1.y);
    y2 = round(vertex2.y);
    y3 = round(other.vertex1.y);
    y4 = round(other.vertex2.y);

    double myNumer = ((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3));
    double otherNumer = ((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3));
    double denom = ((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1));

    if (denom == 0) {
        if (myNumer == 0 || otherNumer == 0) {
            // coincident lines
            // gets the vertex that are facing the same way
            glm::vec4 ptToConsider = (getOrientation() == other.getOrientation()) ? other.getVertex2() : other.getVertex1();
            // intpt is the closer of the my v2 or the ptToConsider. if v2 is further then its out of the boundary and if v2 is closer then the segment ends before the boundary does
            intPt = (glm::distance(getVertex1(), getVertex2()) < glm::distance(getVertex1(), ptToConsider)) ? getVertex2() : ptToConsider;
            return true;
        }
        // parrallel lines
        return false;
    } else {
        double myFrac = myNumer / denom;
        double otherFrac = otherNumer / denom;
        if (myFrac >= 0.0 && myFrac <= 1.0 && otherFrac >= 0.0 && otherFrac <= 1.0) {
            // they intersect some where
            double x = getX1() + (myFrac)* (getX2() - getX1());
            double y = getY1() + (myFrac)* (getY2() - getY1());
            intPt = glm::vec4(x, y, 0.0, 1.0);
            return true;
        }
    }

    intPt = glm::vec4(-1.0, -1.0, -1.0, -1.0);
    intPt.x = round(intPt.x);
    intPt.y = round(intPt.y);
    return false;
}

/**
* This method returns the x cordinet of vertex1.
* @return vertex1.x
*/
double Segment::getX1() const {
    return vertex1.x;
}

/**
* This method returns the x cordinet of vertex2.
* @return vertex2.x
*/
double Segment::getX2() const {
    return vertex2.x;
}

/**
* This method returns the y cordinet of vertex1.
* @return vertex1.y
*/
double Segment::getY1() const {
    return vertex1.y;
}

/**
* This method returns the y cordinet of vertex2
* @return vertex2.y
*/
double Segment::getY2() const {
    return vertex2.y;
}

/**
* This method is used to move the segment. It uses glm::translate and matrix
* multiplacation to modify the x and y cordients the segment's verticies.
* @param dx - The amount to move in the x axis.
* @param dy - The amount to move in the y axis.
*/
void Segment::translate(double dx, double dy) {
    vertex1 = glm::translate(glm::mat4(1.0), glm::vec3(dx, dy, 0.0)) * vertex1;
    vertex2 = glm::translate(glm::mat4(1.0), glm::vec3(dx, dy, 0.0)) * vertex2;
}

/**
* This method is used to rotate the segment. There are two extra parameters
* that are used to change the location of the segment in regards to the origin.
* This is used so a segment is rotated in place at a vertex instead around the
* orign, for example. This is done by moving both verticies by the (dx, dy).
* Then rotating via glm::rotate and matrix multiplication. And lastly moved
* back via (-dx, -dy).
* @param rotationAngle - The amount to rotate the segment by. This is in
*                        degress because glm::rotate is in degrees.
* @param dx            - The amount to move the segment on the x axis.
* @param dy            - The amount to move the segment on the y axis.
*/
void Segment::rotate(double rotationAngle, double dx, double dy) {
    // step 1 move (dx, dy) before rotation
    vertex1 = glm::translate(glm::mat4(1.0), glm::vec3(dx, dy, 0.0)) * vertex1;
    vertex2 = glm::translate(glm::mat4(1.0), glm::vec3(dx, dy, 0.0)) * vertex2;

    // step 2 rotate
    glm::mat4 rotation = glm::rotate(float(rotationAngle), glm::vec3(0.0, 0.0, 1.0));
    vertex1 = rotation * vertex1;
    vertex2 = rotation * vertex2;

    // step 3 move back with (-dx, -dy)
    vertex1 = glm::translate(glm::mat4(1.0), glm::vec3(-dx, -dy, 0.0)) * vertex1;
    vertex2 = glm::translate(glm::mat4(1.0), glm::vec3(-dx, -dy, 0.0)) * vertex2;

    // update the orientation
    orientation = Util::NormAngle(orientation + rotationAngle);
}

/**
* Provides the caller with the first vertex of the segment.
* @return glm::vec4 - A copy of this segment's vertex1.
*/
glm::vec4 Segment::getVertex1() const {
    return glm::vec4(vertex1);
}

/**
* Provides the caller with the second vertex of the segment.
* @return glm::vec4 - A copy of this segment's vertex2.
*/
glm::vec4 Segment::getVertex2() const {
    return glm::vec4(vertex2);
}

/**
* This method is used to determine if the provided segment is coincident to the
* segment calling the method. Two lines are coincident if they are colinear and
* overlap.
* @param other  - The other segment.
* @return true  - Iff the provided segment and this are colinear and
*                 overalpping.
* @return false - Iff the provided segment and this are not colinear or they
*                 are not overalpping.
*/
bool Segment::isCoincident(const Segment& other) const {
    /*    double myNumer = ((other.getX2() - other.getX1()) * (getY1() - other.getY1()))
    - ((other.getY2() - other.getY1()) * (getX1() - other.getX1()));
    double otherNumer = ((getX2() - getX1()) * (getY1() - other.getY1()))
    - ((getY2() - getY1()) * (getX1() - other.getX1()));
    double denom = ((other.getY2() - other.getY1()) * (getX2() - getX1()))
    - ((other.getX2() - other.getX1()) * (getY2() - getY1()));

    */
    double x1, x2, x3, x4, y1, y2, y3, y4;
    x1 = round(vertex1.x);
    x2 = round(vertex2.x);
    x3 = round(other.vertex1.x);
    x4 = round(other.vertex2.x);
    y1 = round(vertex1.y);
    y2 = round(vertex2.y);
    y3 = round(other.vertex1.y);
    y4 = round(other.vertex2.y);

    double myNumer = ((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3));
    double otherNumer = ((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3));
    double denom = ((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1));

    if (denom == 0) {
        if (myNumer == 0 || otherNumer == 0) {
            // coincident lines
            return true;
        }
        // parrallel lines
        return false;
    }
    return false;
}

/**
* This method is used to determine if a given point is located on the line segment.
* http://stackoverflow.com/posts/328337/revisions implementation and had to make cmp() method
* @param pt     - The point in question.
* @return true  - Iff the point is located on the line segment
* @return false - Iff the point is not located on the line segment.
*/
bool Segment::isOn(glm::vec4 &pt) const {
    double aX = getX1(), bX = getX2(), cX = pt.x;
    double aY = getY1(), bY = getY2(), cY = pt.y;

    if ((bX - aX) * (cY - aY) == (cX - aX) * (bY - aY)) {
        double xCompSum = 0.0, yCompSum = 0.0;

        if (aX < cX) {
            xCompSum -= 1.0;
        } else if (aX > cX) {
            xCompSum += 1.0;
        }

        if (bX < cX) {
            xCompSum -= 1.0;
        } else if (bX > cX) {
            xCompSum += 1.0;
        }

        if (aY < cY) {
            yCompSum -= 1.0;
        } else if (aY > cY) {
            yCompSum += 1.0;
        }

        if (bY < cY) {
            yCompSum -= 1.0;
        } else if (bY > cY) {
            yCompSum += 1.0;
        }

        if (std::abs(xCompSum) <= 1.0 && std::abs(yCompSum) <= 1.0) {
            return true;
        }
    }
    return false;
}

/**
* This method is used to determine the distance from a given point and the line segment.
* http://paulbourke.net/geometry/pointlineplane/
* @param pt      - The point in question.
* @return double - The distance from the given point and the line segment.
*/
double Segment::distToPoint(glm::vec4 &pt) const {
    double u = (((pt.x - vertex1.x) * (vertex2.x - vertex1.x)) + ((pt.y - vertex1.y) * (vertex2.y - vertex1.y))) / (std::abs(length * length));
    double x = vertex1.x + (u * (vertex2.x - vertex1.x));
    double y = vertex1.y + (u * (vertex2.y - vertex1.y));
    return glm::distance(pt, glm::vec4(x, y, 0.0, 1.0));
}
