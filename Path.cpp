#include <RedirectedWalking/PRT/Path.h>
#include <exception>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <strstream>

#pragma once

using namespace Hive;

/**
* This is the defualt constructor. The segments vector is empty.
*/
Path::Path(){
    segments = std::vector<Segment>();
}


/**
* This constructor uses the string parameter as the name of the file to read
* Segment data from. The data is stored "x1 y1 x2 y2\n".
* @param fileName - The name of the file to read from.
*/
Path::Path(std::string &fileName) {
    segments.clear();
    std::ifstream pathFile;
    pathFile.open(fileName);

    if (pathFile.is_open()) {
        double x1, x2, y1, y2;
        while (pathFile.good() && !pathFile.eof()) {
            pathFile >> x1;
            pathFile >> y1;
            pathFile >> x2;
            pathFile >> y2;
            Segment tmpSegment1(x1, y1, x2, y2);
            segments.push_back(tmpSegment1);
        }
        pathFile.close();
    } else {
        std::cerr << "Unable to read file: " << fileName << ".\n";
    }
}

/**
* This constructor uses an already existing vector of segments to fill in the
* new instance of Path with a copy of its data.
* @param newSegments - A container that holds segments to copy.
*/
Path::Path(const std::vector<Segment> &newSegments) {
    segments = std::vector<Segment>();
    for (unsigned int i = 0; i < newSegments.size(); ++i) {
        segments.push_back(newSegments[i]);
    }
}

/**
* This constructor is a copy constructor. It copies the other instance's
* segments into the new instance.
* @param other - The instance to copy data from.
*/
Path::Path(const Path &other) {
    segments = std::vector<Segment>(other.numSegments());
    // std::copy(other.segments.begin(), other.segments.end(), segments.begin());
    for (unsigned int i = 0; i < segments.size(); ++i) {
        segments[i] = Segment(other.segments[i]);
    }
}

/**
* This method calculates the length of the path which is the sum of the
* segments' lengths.
* @return sum - The total length of the path.
*/
double Path::calculateLength() const {
    double sum = 0.0;
    std::for_each(segments.begin(), segments.end(),
        [&sum](const Segment &s) {
            sum += s.getLength();
        }
    );
    return sum;
}

/**
* This method returns how many segments are in the path.
* @return segments.size()
*/
unsigned int Path::numSegments() const {
    return segments.size();
}

/**
* This method is used to rotate the path. The method uses the vertex1 of the
* first segment of the segments vector as the 'origin' of the path. So the
* (dx, dy) used is (-1 * vertex1.x, -1 * dy). This moves the path to the true
* origin of (0, 0, 0, 1).
* @param rotationAngle - The amount to rotate the path. This value is in
*                        degrees.
*/
void Path::rotate(double rotationAngle) {
    double dx = (segments.size() > 0) ? segments[0].getX1() * -1.0 : 0.0;
    double dy = (segments.size() > 0) ? segments[0].getY1() * -1.0 : 0.0;

    for (unsigned int i = 0; i < segments.size(); ++i) {
        segments[i].rotate(rotationAngle, dx, dy);
    }
}

/**
* This method is used to rotate a path to a specified angle. The method
* determiens the difference between its current orientation and the desired
* angle. Then the path is turned to that angle measure.
* @param targetAngle - The angle that the path will be rotated to face.
*/
void Path::rotateToFace(double targetAngle) {
    targetAngle = Util::NormAngle(targetAngle);
    double deltaAngle = 0;
    if (segments.size() > 0) {
        deltaAngle = targetAngle - segments[0].getOrientation();
    }
    rotate(deltaAngle);
}

/**
* This method is used to move the path. The path is moved by moving the
* indivdual segments.
* @param dx - The amount to move the path on the x axis.
* @param dy - The amount to move the path on the y axis.
*/
void Path::translate(double dx, double dy) {
    for (unsigned int i = 0; i < segments.size(); ++i) {
        segments[i].translate(dx, dy);
    }
}

/**
* This method is used to translate the vw to the position provided. This will
* allow the user's vw, on the skeleton, and rw locations to be represented by
* the physical location. The entire path's verticies are translated by <dx,dy>.
* @param x - The target x position.
* @param y - The target y position.
*/
void Path::center(double x, double y) {
    double dx = 0.0;
    double dy = 0.0;

    if (segments.size() != 0) {
        dx = x - segments[0].getX1();
        dy = y - segments[0].getY1();
    }

    translate(dx, dy);
}

/**
* This method is used to get the difference in two of the path's segments'
* orientation. This is commonly used on segs[i] and segs[i + 1]. There is an
* assumption that 0 <= i < |segments| and 0 <= j < |segments|.
* @param i - The index of a segment.
* @param j - The index of another segment.
* @return segs[j].* - segs[i].*
*/
double Path::getChangeInOrientation(unsigned int i, unsigned int j) const {
    double startingAngle = segments[i].getOrientation();
    double endingAngle = segments[j].getOrientation();
    return endingAngle - startingAngle;
}

/**
* This method provides the caller with a copy of the segment that is located at
* location index. Assumed that 0 <= index < |segments|.
* @param index - The indicie of the segment to provide the caller.
* @return A deep copy of segs[index]
*/
Segment Path::getSegment(unsigned int index) const {
    return Segment(segments[index]);
}

/**
* This method adds a segment to the front of the path. It will be placed at
* segs[0] and all the other segments will be 'pushed' back one index. The
* method does this by creating a new vector and adding the newSegment to it.
* Then the segments in the path's vector are added to the vector. After that
* the new vector is then assigned as the vector of the path.
* @param newSegment - The segment that will be added to the path's vector.
*/
void Path::addSegmentToFrontOfPath(const Segment &newSegment) {
    std::vector<Segment> segs;
    segs.push_back(Segment(newSegment));
    for (unsigned int i = 0; i < segments.size(); ++i) {
        segs.push_back(Segment(segments[i]));
    }
    segments.clear();
    segments = std::vector<Segment>(segs);
}

/** This method is used to get the distance from the first intersection (the
* point of reset) and the second reset the furthest the user could go. Each
* segment in the path is checked if it intersects with the boundary, aka other.
* If it intersects then the distance is from the start of the path to the
* intersection point. Other wise check the next segment. If there is no
* intersection then we project three segments out from the end of the path and
* add their length, no intersection, or dist to intersect. We asume that the
* starting point of the path, reset location, is inside of the boundary, other.
* @param other - The boundary to check for intersections with.
* @return dist - The total distance that the user could walk before having
*                another reset.
*/
double Path::getDistanceToSecondIntersection(const Path &other) const {
    double dist = 0.0;
    bool intersected = false;
    glm::vec4 intPt;

    for (unsigned int i = 0; i < segments.size() && !intersected; ++i) {
        for (unsigned int j = 0; j < other.segments.size() && !intersected; ++j) {
            if (segments[i].isCoincident(other.segments[j])) {
                intersected = true;
                // we want the closer one.
                double distToV2 = glm::distance(segments[i].getVertex1(), segments[i].getVertex2());
                glm::vec4 otherPt;
                if (std::abs(segments[i].getOrientation() - other.segments[j].getOrientation()) < 0.2) { // orientations the same
                    //other pt = other.v2
                    otherPt = other.segments[j].getVertex2();
                } else {
                    //other pt = other.v1
                    otherPt = other.segments[j].getVertex1();
                }
                double distToWallPt = glm::distance(segments[i].getVertex1(), otherPt);
                dist += std::min(distToWallPt, distToV2);
            } else if (segments[i].hasIntersection(other.segments[j], intPt)) {
                if (intPt != segments[i].getVertex1()) {
                    // it means that [i] intersected at two places. the reset point and the other location
                    intersected = true;
                    dist += glm::distance(intPt, segments[i].getVertex1());
                }
            }
        }
        // if there was no intersection
        if (!intersected) {
            dist += segments[i].getLength();
            // At the end and b/c no intersection therefor there must be some more room to move
            if (i == segments.size() - 1) {
                double a1 = 1000.0 * -std::sin(segments[i].getOrientation()- (90.0));
                double o1 = 1000.0 * std::cos(segments[i].getOrientation() - (90.0)); // right turn
                double a2 = 1000.0 * -std::sin(segments[i].getOrientation());
                double o2 = 1000.0 * std::cos(segments[i].getOrientation()); // straight
                double a3 = 1000.0 * -std::sin(segments[i].getOrientation() + (90.0));
                double o3 = 1000.0 * std::cos(segments[i].getOrientation() + (90.0)); // left

                double xP = segments[i].getX2(), yP = segments[i].getY2();
                double x1, x2, x3, y1, y2, y3;

                x1 = xP + a1; y1 = yP + o1;
                x2 = xP + a2; y2 = yP + o2;
                x3 = xP + a3; y3 = yP + o3;

                Segment s1 = Segment(xP, yP, x1, y1);
                Segment s2 = Segment(xP, yP, x2, y2);
                Segment s3 = Segment(xP, yP, x3, y3);

                std::vector<Segment> segs;
                segs.push_back(s1);
                segs.push_back(s2);
                segs.push_back(s3);

                double myDist = 0.0;
                for (unsigned int j = 0; j < segs.size(); ++j) {
                    bool iIntersected = false;
                    for (unsigned int k = 0; k < other.segments.size() && !iIntersected; ++k) {
                        if (segs[j].isCoincident(other.segments[k])) {
                            iIntersected = true;
                            // we want the closer one.
                            double distToV2 = glm::distance(segs[j].getVertex1(), segs[j].getVertex2());
                            glm::vec4 otherPt;
                            if (std::abs(segs[j].getOrientation() - other.segments[k].getOrientation()) < 0.2) { // orientations the same
                                //other pt = other.v2
                                otherPt = other.segments[k].getVertex2();
                            } else {
                                //other pt = other.v1
                                otherPt = other.segments[k].getVertex1();
                            }
                            double distToWallPt = glm::distance(segs[j].getVertex1(), otherPt);
                            myDist += std::min(distToWallPt, distToV2);
                        } else if (segs[j].hasIntersection(other.segments[k], intPt)) {
                            if (intPt != segs[j].getVertex1()) {
                                // it means that [j] intersected at two places. the end point and the other location
                                iIntersected = true;
                                myDist += glm::distance(intPt, segs[j].getVertex1());
                            }
                        } else {
                            myDist += segs[j].getLength();
                        }
                    }
                }
                dist += myDist / ((double)segs.size()); // average here
            }
        }
    }

    return dist;
}

/**
* This method is used to change the starting vertext of the path. That way the
* path starts at a reset lcoation instead of the skeleton's segment's vertex 1.
* @param newPt - The new vertex.
* THE OTHER PARAMS WERE USED IN OLD PRT SIMULATIONS AND WERE NOT REMOVED FROM
* THE METHOD'S SIGNITURE.
*/
void Path::set0thV1(glm::vec4 &newPt, double maxX, double minX, double maxY, double minY) {
    if (segments.size() > 0) {
        double x = newPt.x;
        double y = newPt.y;
        Segment replacement = Segment(x, y, segments[0].getX2(), segments[0].getY2());
        segments[0] = replacement;
    }
}

/**
* This method is used to create a string representation of the path. The
* represenation is vertex 0 -> vertex 1 -> ... -> vertex n.
* @return vertex 0 -> ... -> vertex n
*/
std::string Path::toString() const {
    std::strstream output;
    glm::vec4 v1 = getSegment(0).getVertex1();
    glm::vec4 v2 = getSegment(0).getVertex2();
    output << "(" << v1.x << "," << v1.y << ")\t ->\t(" << v2.x << "," << v2.y << ")";

    for (unsigned int i = 1; i < segments.size(); ++i) {
        // only care about v2 b/c v1 is last segment's v1
        v2 = getSegment(i).getVertex2();
        output << "\t ->\t(" << v2.x << ", " << v2.y << ")";
    }

    return output.str();
}
