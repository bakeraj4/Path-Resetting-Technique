#include <RedirectedWalking/PRT/Boundary.h>
#include <exception>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#pragma once

using namespace Hive;

/**
* The default constructor. The segments vector is empty.
*/
Boundary::Boundary() : Path() {
}

/**
* This constructor uses the string parameter as the name of the file to read
* Segment data from. The data is stored "x1 y1 x2 y2\n".
* @param fileName - The name of the file to read from.
*/
Boundary::Boundary(std::string &fileName) : Path(fileName) {
}


/**
* This constructor uses an already existing vector of segments to fill in the
* new instance of Path with a copy of its data.
* @param newSegments - A container that holds segments to copy.
*/
Boundary::Boundary(const std::vector<Segment> &newSegments) : Path(newSegments) {
}

/**
* This constructor is a copy constructor. It copies the other instance's
* segments into the new instance.
* @param other - The instance to copy data from.
*/
Boundary::Boundary(const Boundary &other) : Path(other) {
}

/**
* This method is used to determine if a given point is out side of the
* boundary. Because the room is assumed to be rectangular we can assume that if
* a point's x value is less than the minimum x value or larger than the maximum
* x value then the point is out side. And the same holds true for y values. The
* point is inside iff for both x and y that the previous two statements are
* false. The min and maximum functions are not used because they each look at
* all of the segments. Only using one pass is better and in this pass all of
* the min/max's are updated.
* @param  loc   - The location in question.
* @return true  - The point in question lays out side of the boundary.
* @return false - The point in question lays in side of the boundary.
*/
bool Boundary::outOfBounds(glm::vec4 loc) const {
    double minX = std::numeric_limits<double>::max();
    double maxX = -1 * std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -1 * std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < segments.size(); ++i) {
        // update the min x value
        if (minX > segments[i].getX1()) {
            minX = segments[i].getX1();
        }
        if (minX > segments[i].getX2()) {
            minX = segments[i].getX2();
        }
        // update the max x value
        if (maxX < segments[i].getX1()) {
            maxX = segments[i].getX1();
        }
        if (maxX < segments[i].getX2()) {
            maxX = segments[i].getX2();
        }

        // update the min y value
        if (minY > segments[i].getY1()) {
            minY = segments[i].getY1();
        }
        if (minY > segments[i].getY2()) {
            minY = segments[i].getY2();
        }

        if (maxY < segments[i].getY1()) {
            maxY = segments[i].getY1();
        }
        if (maxY < segments[i].getY2()) {
            maxY = segments[i].getY2();
        }
    }

    if (loc.x < minX) { // &&  std::abs(loc.x - minX) > ERR) {
        return true; // out of bounds to the left
    }

    if (loc.x > maxX) { //  && std::abs(loc.x - maxX) > ERR) {
        return true; // out of bounds to the right
    }

    if (loc.y < minY) { //  && std::abs(loc.y - minY) > ERR) {
        return true; // out of bounds to the bottom
    }

    if (loc.y > maxY) { //  && std::abs(loc.y - maxY) > ERR) {
        return true; // out of bounds to the top
    }

    return false;
}

/**
* This method is used to get the minimum x value of the boundary. Because each
* pair of segments (i, i+1) share an endpoint only one endpoint of each segment
* needs to be checked. Because a rectangular room is assumed this will be the x
* value of the western wall.
* @return min - The minimum x value of all the boundary's segments.
*/
double Boundary::getMinX() const {
    double min = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (min > segments[i].getVertex2().x) {
            min = segments[i].getVertex2().x;
        }

    }
    return min;
}

/**
* This method is used to get the maximum x value of the boundary. Because each
* pair of segments (i, i+1) share an endpoint only one endpoint of each segment
* needs to be checked. Because a rectangular room is assumed this will be the x
* value of the eastern wall.
* @return max - The maximum x value of all the boundary's segments.
*/
double Boundary::getMaxX() const {
    double max = std::numeric_limits<double>::min();
    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (max < segments[i].getVertex2().x) {
            max = segments[i].getVertex2().x;
        }

    }
    return max;
}

/**
* This method is used to get the minimum y value of the boundary. Because each
* pair of segments (i, i+1) share an endpoint only one endpoint of each segment
* needs to be checked. Because a rectangular room is assumed this will be the y
* value of the southern wall.
* @return min - The minimum y value of all the boundary's segments.
*/
double Boundary::getMinY() const {
    double min = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (min > segments[i].getVertex2().y) {
            min = segments[i].getVertex2().y;
        }

    }
    return min;
}

/**
* This method is used to get the maximum y value of the boundary. Because each
* pair of segments (i, i+1) share an endpoint only one endpoint of each segment
* needs to be checked. Because a rectangular room is assumed this will be the y
* value of the northern wall.
* @return max - The maximum y value of all the boundary's segments.
*/
double Boundary::getMaxY() const {
    double max = std::numeric_limits<double>::min();
    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (max < segments[i].getVertex2().y) {
            max = segments[i].getVertex2().y;
        }

    }
    return max;
}
