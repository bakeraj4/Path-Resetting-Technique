#include <RedirectedWalking/PRT/SegmentSkeleton.h>
#include <exception>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#pragma once

using namespace Hive;

/**
* Default constructor that is used to create empty container.
*/
SegmentSkeleton::SegmentSkeleton() {
    segments = std::vector<Segment>();
}

/**
* Constructor used to create container and fill it with segments read in from
* the provided file.
* @param skelFile - The file name to read from.
*/
SegmentSkeleton::SegmentSkeleton(std::string &skelFile) {
    segments = std::vector<Segment>();
    segments.clear();
    std::ifstream segs;
    segs.open(skelFile);

    if (segs.is_open()) {
        double x1, x2, y1, y2;
        while (segs.good() && !segs.eof()) {
            segs >> x1;
            segs >> y1;
            segs >> x2;
            segs >> y2;
            // adding both ways allows the skeleton to be thought of as a bi directional graph
            Segment tmpSegment1(x1, y1, x2, y2);
            segments.push_back(tmpSegment1);
            Segment tmpSegment2(x2, y2, x1, y1);
            segments.push_back(tmpSegment2);
        }
        segs.close();
    }
    else {
        std::cerr << "Unable to read file: " << skelFile << ".\n";
    }
}

/**
* This method is used to rotate the skeleton. The delta x and y are needed so
* the skeleton is rotate at the correct orign. Other wise the skeleton is
* rotated around the (0,0) orign and places the skeleton in the wrong location.
* @param rotationAngle - The angle measure to rotate the skeleton by.
* @param dx            - The x offset from the orign to rotate around.
* @param dy            - The y offset from the orign to rotate around.
*/
void SegmentSkeleton::rotate(double rotationAngle, double dx, double dy) {
    dx *= -1.0;
    dy *= -1.0;

    for (unsigned int i = 0; i < segments.size(); ++i) {
        segments[i].rotate(rotationAngle, dx, dy);
    }
}

/**
* This method is used to move the skeleton.
* @param dx - The amount to move on the x axis.
* @param dy - The amount to move on the y axis.
*/
void SegmentSkeleton::translate(double dx, double dy) {
    for (unsigned int i = 0; i < segments.size(); ++i) {
        segments[i].translate(dx, dy);
    }
}

/**
* This method provides the caller with a copy of the segment located at the
* provided index. The index is assumed to be in th range [0, segments.size()).
* @param index    - The index of the segment to get.
* @return Segment - A copy of the segment at the location index of the
*                   skeleton's vector of segments.
*/
Segment SegmentSkeleton::getSegment(unsigned int index) {
    return Segment(segments[index]);
}

/**
* This method gets the segments that have v1's that are the same as the
* segment[index]'s v2.
* @param  index           - The index of the segment that the user is
*                           currently at.
* @return vector<Segment> - A list of segments that are neighbors.
*/
std::vector<Segment> SegmentSkeleton::getNeighbors(unsigned int index) const {
    std::vector<Segment> neighors;

    Segment currentSegment = Segment(segments[index]);

    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (currentSegment.getVertex2() == segments[i].getVertex1()) {
            // possibly: curr -> neighbor or curr -> curr
            neighors.push_back(Segment(segments[i]));
        }
    }

    return neighors;
}

/**
* This method gets the segments indicies in the segments vector that have v1's
* that are the same as the segment[index]'s v2.
* @param  index           - The index of the segment that the user is
*                           currently at.
* @return vector<unsigned> - A list of segments' indicies that are neighbors.
*/
std::vector<unsigned int> SegmentSkeleton::getNeighborsIndicies(unsigned int index) const {
    std::vector<unsigned int> neighors;

    Segment currentSegment = Segment(segments[index]);

    unsigned int matches = 0;

    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (currentSegment.getVertex2() == segments[i].getVertex1()) {
            ++matches;
        }
    }

    for (unsigned int i = 0; i < segments.size(); ++i) {
        if (currentSegment.getVertex2() == segments[i].getVertex1()) {
            // possibly: curr -> neighbor or curr -> curr
            if ((matches == 1) || (matches > 1 && currentSegment.getVertex1() != segments[i].getVertex2())) {
                // no back tracking, unless its the only option
                neighors.push_back(i);
            }
        }
    }

    return neighors;
}

/**
* This method is used to create paths that the user could possibly go down. The
* limiting factor is how many segments make up the path. This method also
* returns a corresponding percentage that correlates to how likely the user is
* to walk down it.
* @param maxNumSegments  - The number of remaing segments to use.
* @param currentLocation - The index of the user's current segment.
* @param userLoc         - The physical location of the user.
* @param isRoot          - Wether or not the method is the intial call or is a
*                          later call.
* @return paths          - Contains the created paths.
*/
std::vector<std::pair<Path, double>> SegmentSkeleton::generatePathsAndPercents(unsigned int maxNumSegments, unsigned int currentLocation, glm::vec4 &userLoc, bool isRoot, const Boundary &bound) const {
    std::vector<std::pair<Path, double>> ret;

    Segment seg;
    if (isRoot) {
        seg = Segment(userLoc.x, userLoc.y, segments[currentLocation].getX2(), segments[currentLocation].getY2());
    } else {
        seg = Segment(segments[currentLocation]);
    }

    if (maxNumSegments <= 0) {
        std::vector<Segment> segs = std::vector<Segment>(1, seg);
        ret.push_back(std::pair<Path, double>(Path(segs), 1.0));
        return ret;
    } else {
        std::vector<unsigned int> neighbors = getNeighborsIndicies(currentLocation);
        double numPushed = 0.0;

        for (unsigned int i = 0; i < neighbors.size(); ++i) {
            std::vector<std::pair<Path, double>> neighborPathsAndPercents = generatePathsAndPercents(maxNumSegments - 1, neighbors[i], userLoc, false, bound);
            for (unsigned int j = 0; j < neighborPathsAndPercents.size(); ++j) {
                Path p = Path(neighborPathsAndPercents[j].first);

                Segment lastSeg = Segment(p.getSegment(0));

                if (seg.getVertex2() == lastSeg.getVertex1()) {
                    numPushed += 1.0;
                    p.addSegmentToFrontOfPath(seg);
                    ret.push_back(std::pair<Path, double>(p, 1.0));
                }
            }
        }
        if (ret.size() == 0) {
            return ret;
        } else {
            double percent = 1.0 / numPushed;
            for (unsigned int k = 0; k < ret.size(); ++k) {
                ret[k].second *= percent;
            }
        }
        return ret;
    }
}


/**
* This method gives the number of segments in the skeleton.
* @return unsigned int - The size of the segments vector.
*/
unsigned int SegmentSkeleton::getNumSegments() const {
    return segments.size();
}
