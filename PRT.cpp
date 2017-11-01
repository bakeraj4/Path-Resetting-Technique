#include <RedirectedWalking/PRT/PRT.h>
#include <exception>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "Utils\outfile.h"
#include <ostream>
#pragma once

using namespace Hive;

/**
* Creates an instance of the PRT class.
* @param config - Contains data to setup PRT.
*/
PRT::PRT(Config config) : ResettingAlgorithm(config) {
    isArbitrary = config.get("mode", false);

    // read real world
    // The file used here needs to model the 'safte' walking area.
    // The deault HIVE walking area is -12.5 to 12.5 in x and -22 to 22 in y
    // So, use 
    realWorld = Boundary(std::string(config.get("realWorld", "defaultGym.txt")));

    // read virtual world
    virtualWorld = SegmentSkeleton(std::string(config.get("virtualWorld", "defaultWorld.txt")));

    // get the starting segment
    currentSegment = config.get("startingSegment", 0);

    bool firstUpdate = true;
}

/**
* This method is used to determine the best angle the user could face after the
* reset. The user's <x, y> position is clamped to the boundary, safe walking
* space, before the calculations are performed.The method gets the min and max
* angle to use and adds all angles between into a vector. The angle that has
* the highest score is chosen as the angle to face. The gain factor to turn is
* calculated.
* @param x - The x position of the reset.
* @param y - The y position of the reset.
* relativeToNorth is not used.
*/
void PRT::reset(double x, double y, double relativeToNorth) {
    std::vector<double> anglesToConsider;

    // I am clamping the user's x, y pos. This way they are going to be in the
    // boundary and not be directed out of it.
    if (x > W / 2.0 - buffer) {
        x = W / 2.0 - buffer;
    } else if (x < -W / 2.0 + buffer) {
        x = -W / 2.0 + buffer;
    }

    if (y > H / 2.0 - buffer) {
        y = H / 2.0 - buffer;
    } else if (y < -H / 2.0 + buffer) {
        y = -H / 2.0 + buffer;
    }


    // The vw has to have some adjustments before it is ready to be used,
    // because the user can move across the width of the aisle they will not be
    // exactly on the segment. The changes are in (x, y). The angle is handled
    // in update()
    glm::vec4 intPt(x, y, 0.0, 1.0);

    double dx = x - intPt.x;
    double dy = y - intPt.y;
    virtualWorld.translate(dx, dy);

    for (unsigned int i = 0; i < realWorld.numSegments(); ++i) {
        if (realWorld.getSegment(i).isOn(glm::vec4(x, y, 0.0, 1.0))) { // user is on the line of the boundary
            if (realWorld.getSegment(i).hasIntersection(virtualWorld.getSegment(currentSegment), intPt)) { // path intersects it            
                i = realWorld.numSegments() + 1; // breaking out of the loop
            }
        }
    }


    // add the relative angle from the user to the corners, only add those that iia would
    for (unsigned int i = 0; i < realWorld.numSegments(); ++i) {
        glm::vec4 pt = realWorld.getSegment(i).getVertex2();
        double angle = Util::NormAngle(Util::R2D(-std::atan2(pt.x - x, pt.y - y)));
        if (!isAngleInCone(angle)) {
            anglesToConsider.push_back(angle);
        }
    }

    // add the rear angles that IIA uses
    addRearAngles(anglesToConsider);

    // add the remaining angles
    addAngles(determineMinMax(x, y), anglesToConsider);

    // normalize the angles
    for (unsigned int i = 0; i < anglesToConsider.size(); ++i) {
        anglesToConsider[i] = Util::NormAngle(anglesToConsider[i]);
    }

    // get the best angle
    bestAngle = determineBestAngle(anglesToConsider);

    // set the gain and direction of rotation.
    // The ResettingAlg's checkSpin() ensure's that the user rotates this much
    CalculateGain(bestAngle - HPR[0]);

#ifdef TrialLog
    log(true);
#endif
}

/**
* This method adds angles between min and max to a vector.
* @param minMax - The minimum, minMax.fisrt, and maximum, minMax.second.
* @param anglesToConsider - Angles will be added to this vector
*/
void PRT::addAngles(std::pair<double, double> &minMax, std::vector<double> &anglesToConsider) const {
   bool addedMax = false; 
   for (double alpha = minMax.first; alpha <= minMax.second; alpha += PRT_ANGLE_DELTA) {
        if (!isArbitrary && !isAngleInCone(alpha)) {
            anglesToConsider.push_back(alpha);
            if (alpha == minMax.second) {
                addedMax = true;
            }
        } else if (isArbitrary) {
            anglesToConsider.push_back(alpha);
            if (alpha == minMax.second) {
                addedMax = true;
            }
        }
    }

    if (!addedMax) {
        if (!isArbitrary && !isAngleInCone(minMax.second)) {
            anglesToConsider.push_back(minMax.second);
        } else if (isArbitrary) {
            anglesToConsider.push_back(minMax.second);
        }
    }
}

/**
* This method is used to determine the min and max angles to consider. It uses
* The person's location to put it into one of the 9 catergories and choses
* acordingly.
* @param x - The x position of the user's reset location
* @param y - The y position of the user's reset location
* @return <min, max>
*/
std::pair<double, double> PRT::determineMinMax(double x, double y) const {
    // these two sets of vars barrowed from IIA

    // get the current left most, right most, bottom most, top most position
    double left = x < leftSaftey ? x : leftSaftey;
    double right = x > rightSaftey ? x : rightSaftey;
    double bottom = y < bottomSaftey ? y : bottomSaftey;
    double top = y > topSaftey ? y : topSaftey;
   
    bool closeToLeft = Util::CloseTo(x, left);
    bool closeToRight = Util::CloseTo(x, right);
    bool closeToTop = Util::CloseTo(y, top);
    bool closeToBottom = Util::CloseTo(y, bottom);

    double min, max;
    /**
    * The following cases are based on the diagrams in my notes.
    *   A   |   H   |   G
    *   __________________
    *   B   |   I   |   F
    *   __________________
    *   C   |   D   |   E
    */
    // The min have added 5* and the max has subtracted 5*. This was to attempt
    // to reduce the chance of walking parrallel to the buffer.
    if (closeToLeft && closeToTop) { // A: top left corner 
        min = 185.0; //180.0;
        max = 265.0; //270.0;
    } else if (closeToLeft && closeToBottom) { // C: bottom left
        min = 275.0; //270.0;
        max = 355.0; //360.0;
    } else if (closeToRight && closeToBottom) { // E: bottom right corner
        min = 5.0; ///0.0;
        max = 85.0; //90.0;
    } else if (closeToRight && closeToTop) { // G: top right corner
        min = 95.0; //90.0;
        max = 175.0; //180.0;
    } else if (closeToLeft) { // B: left
        min = 185.0; //180.0;
        max = 355.0; //360.0;
    } else if (closeToBottom) { // D: bottom
        min = 275.0; //270.0;
        max = 445.0; //450.0;
    } else if (closeToRight) { // F: right
        min = 5.0; //0.0;
        max = 175.0; //180.0;
    } else if (closeToTop) { // H: top
        min = 95.0; //90.0;
        max = 265.0; //270.0;
    } else { // I: middle
        min = 0.0;
        max = 360.0;
    }
    return std::pair<double, double>(min, max);
}

/**
* This method is called by the pipeline to refresh the data that the algorithm
* will use during a reset. The alg's representation of the vw is rotated as
* needed and queries the unity side (if not a simulation) about the segment
* that the user is on. The parent update handles what it should be doing as a 
* part of the pipeline.
*/
void PRT::update(DataContainer &dataContainer, const double &elapsedTime,
    HeadingOffset &headingOffset) {

    this->inputData = dataContainer;
    this->headingOffset = headingOffset;
    this->elapsedTime = elapsedTime;

    // getting the change in rotation before the update b/c prevYaw is overwritten in a method called by ResettingAlg::update
    double lastYaw = prevYaw;

    this->ResettingAlgorithm::update();

    double dTheta = prevYaw - lastYaw; // resetAlg::update() updates prevYaw

    // in the first update the virtual world needs to be overlayed on top of
    // the user.
    if (firstUpdate) {
        firstUpdate = false;
        // determine the offset needed
        double dx, dy;
        dx = pos.x - virtualWorld.getSegment(currentSegment).getVertex1().x;
        dy = pos.y - virtualWorld.getSegment(currentSegment).getVertex1().y;

        // the actual overlay
        virtualWorld.translate(dx, dy);
        // turning the vww to match the user's forward facing
        virtualWorld.rotate(HPR[0] - virtualWorld.getSegment(currentSegment).getOrientation(), dx, dy);
#ifdef TrialLog
        log(false);
#endif
    } else {
        // the user pos is subtracted out so the vw is spun around the origin
        virtualWorld.rotate(dTheta, pos.x, pos.y);
    }

    unsigned int nextSegment = currentSegment;

    // get the new starting segment
    inputData.get(SEGMENT_NUMBER, nextSegment);

    // the unity side handles if on ab or ba
    // rotate the vw the difference between the two segments
    if (nextSegment != currentSegment) { // on different segments. natural head turning will handle turning onto different legs of a shape
        Segment prev = virtualWorld.getSegment(currentSegment);
        Segment curr = virtualWorld.getSegment(nextSegment);
        if (prev.getVertex1() == curr.getVertex2() && prev.getVertex2() == curr.getVertex1()) { // if opposite, ie 0 and 1
            double delta = virtualWorld.getSegment(nextSegment).getOrientation() - virtualWorld.getSegment(currentSegment).getOrientation();
            virtualWorld.rotate(delta, pos.x, pos.y);
       }
    }
    currentSegment = nextSegment;

    outputData.set("SEGMENT_NUMBER", currentSegment);

    headingOffset = this->headingOffset;
    dataContainer = this->outputData;
}

/**
* This method is used to determine if an angle is in the cone. Meaning would
* that angle be perceptable if we turned to it?
* @param angle - The angle to turn to that is in question
* @return true - iff the angle is in the cone
* @return false - iff the anlge is out of the cone 
*/
bool PRT::isAngleInCone(double angle) const {
    double userHeading = HPR[0];
    double rearAngleA = userHeading + firstRearAngle;
    rearAngleA = Util::NormAngle(rearAngleA);
    double userX = pos.x, userY = pos.y;

    double rayAEndptX = -std::sin(Util::D2R(rearAngleA)) * 100.0 + userX;
    double rayAEndptY = std::cos(Util::D2R(rearAngleA)) * 100.0 + userY;

    // actually make the rayA
    Segment rayA = Segment(userX, userY, rayAEndptX, rayAEndptY);

    // adjust for cordinate frame
    rearAngleA = Util::NormAngle(rayA.getOrientation());
    double rearAngleB = rearAngleA + angleOfRearUnreachableCone;

    angle = Util::NormAngle(angle);

    if (angle < rearAngleA) {
        angle += 360.0; // when rearAngleA gets large like when at right wall A = ~ 355 and then top left corner ~46 is in the cone and method says it is
    }

    // if angle is equal to either rear angle that is ok b/c that is the extent
    // of what the user can turn to before being precieved.
    return (rearAngleA < angle) && (angle < rearAngleB);
}

/**
* This method scores the angles and chooses the best one. The angle that is the
* best allows the user to walk the furthest.
* @param angles - A vector that contains the possible angles that the user
*                 could be facing after the reset.
* @return bestAngle - The angle that score the best and will be the orientation
*                     that the user will now face.
*/
double PRT::determineBestAngle(const std::vector<double> &angles) {
    std::vector<std::pair<Path, double>> paths = virtualWorld.generatePathsAndPercents((unsigned int)NUM_SEGS, currentSegment, glm::vec4(pos.x, pos.y, 0.0, 1.0), true, realWorld);

    double x = pos.x, y = pos.y;

    // I am clamping the user's x, y pos. This way they are going to be in the
    // boundary and not be directed out of it.
    if (x > W / 2.0 - buffer) {
        x = W / 2.0 - buffer;
    }
    else if (x < -W / 2.0 + buffer) {
        x = -W / 2.0 + buffer;
    }

    if (y > H / 2.0 - buffer) {
        y = H / 2.0 - buffer;
    }
    else if (y < -H / 2.0 + buffer) {
        y = -H / 2.0 + buffer;
    }
    // make the user's point
    for (unsigned int i = 0; i < paths.size(); ++i) {
        paths[i].first.set0thV1(glm::vec4(x, y, 0.0, 1.0), realWorld.getMaxX(), realWorld.getMinX(), realWorld.getMaxY(), realWorld.getMinY());
    }

    double bestScore = -1.0;
    bestAngle = angles[0];

    for (unsigned int i = 0; i < angles.size(); ++i) {
        double myScore = 0.0;

        // go through the paths and score them
        for (unsigned int j = 0; j < paths.size(); ++j) {
            // determine how much to turn
            double angleToTurn = angles[i] - paths[j].first.getSegment(0).getOrientation();

            // rotate it
            paths[j].first.rotate(angleToTurn);

            // determine the distance to the second intersection and multiply by its weight
            double percent = paths[j].second;
            double dist = paths[j].first.getDistanceToSecondIntersection(realWorld);
            myScore += percent * dist;

            // rotate it back
            paths[j].first.rotate(-angleToTurn);
        }

        if (myScore > bestScore) {
            bestAngle = angles[i];
            bestScore = myScore;
        }
    }

#ifdef ResetLog
    detailedLogForReset(paths);
#endif

    return bestAngle;
}

/**
* This method adds the rear angles to the vector of angles that will be
* considered. The angles are added so PRT will have the same information that
* IIA has. The rear angles are the outer boundary of the cone.
* @param angles - The vector that contains the resulting angles to consider.
*/
void PRT::addRearAngles(std::vector<double> &angles) const {
    static double PROJECTION_DIST = 1000.0;
    // Get the user's direction
    double userHeading = HPR[0];

    // perfrom the calc if the user in a normal coridiate frame for the cos/sin
    double rearAngleA = userHeading + firstRearAngle;
    rearAngleA = Util::NormAngle(rearAngleA);
    double rearAngleB = Util::NormAngle(rearAngleA + angleOfRearUnreachableCone);

    // get the user's location in the gym
    double userX = pos.x, userY = pos.y;

    // I am clamping the user's x, y pos. This way they are going to be in the
    // boundary and not be directed out of it.
    if (userX > W / 2.0 - buffer) {
        userX = W / 2.0 - buffer;
    } else if (userX < -W / 2.0 + buffer) {
        userX = -W / 2.0 + buffer;
    }

    if (userY > H / 2.0 - buffer) {
        userY = H/ 2.0 - buffer;
    } else if (userY < -H / 2.0 + buffer) {
        userY = -H / 2.0 + buffer;
    }

    double rayAEndptX = -std::sin(Util::D2R(rearAngleA)) * PROJECTION_DIST + userX;
    double rayAEndptY = std::cos(Util::D2R(rearAngleA)) * PROJECTION_DIST + userY;

    double rayBEndptX = -std::sin(Util::D2R(rearAngleB)) * PROJECTION_DIST + userX;
    double rayBEndptY = std::cos(Util::D2R(rearAngleB)) * PROJECTION_DIST + userY;

    // actually make the rays
    Segment rayA = Segment(userX, userY, rayAEndptX, rayAEndptY);
    Segment rayB = Segment(userX, userY, rayBEndptX, rayBEndptY);

    // find out where ray a intersects boundary with
    glm::vec4 intPtA;
    for (unsigned int i = 0; i < realWorld.numSegments(); ++i) {
        if (rayA.hasIntersection(realWorld.getSegment(i), intPtA) && glm::distance(intPtA, glm::vec4(userX, userY, 0.0, 1.0)) > ERR && intPtA != glm::vec4(userX, userY, 0.0, 1.0)) {
            angles.push_back(rayA.getOrientation());
            break;
        }
    }

    // find out where ray b intersects boundary with
    glm::vec4 intPtB;
    for (unsigned int i = 0; i < realWorld.numSegments(); ++i) {
        if (rayB.hasIntersection(realWorld.getSegment(i), intPtB) && glm::distance(intPtB, glm::vec4(userX, userY, 0.0, 1.0)) > ERR && intPtB != glm::vec4(userX, userY, 0.0, 1.0)) {
            angles.push_back(rayB.getOrientation());
            break;
        }
    }
}

/**
* This method is used to gather data for debugging. It adds HPR and resulting
* HPR to a file for a run.
* @param atReset - true adds the resulting info
* @param atReset - only the origonal HPR, ie starting data
*/
#ifdef TrialLog
void PRT::log(bool atReset) {
    static bool fileSetUp = false;
    std::ofstream myFile;
    if (!fileSetUp ) {
        fileSetUp = true;
        logFile = std::string("PRT_LOG_") + Hive::getDateString();
        myFile = std::ofstream(logFile + ".txt");
    }

    myFile = std::ofstream(logFile + ".txt", std::ios::app);
    myFile << pos << "\t" << HPR;
    if (atReset) {
        myFile << "\t" << bestAngle << "\t" << gain << "\t" << ccw;
    }
    myFile << "\n";
}
#endif

/**
* This method is used to make a debugging file for a single reset. It prints
* the input and results.
* @param paths - The paths that the user could walk. Are printed in the file.
*/
#ifdef ResetLog
void PRT::detailedLogForReset(std::vector<std::pair<Path, double>> &paths) const {
    std::stringstream strstr;
    strstr << "RESET_NUM" << numOfCollisions << "Date" << Hive::getDateString() << "Time" << getTimeString() << ".txt";
    std::string fileName =  strstr.str();
    std::ofstream resetFile(fileName);

    DataContainer tmp = DataContainer(inputData);

    Vector3 p_pos, v_pos;
    tmp.get(PHY_POS, p_pos);
    tmp.get(VR_POS, v_pos);

    Quaternion v_ori, p_ori;
    tmp.get(PHY_ORI, p_ori);
    tmp.get(VR_ORI, v_ori);

    resetFile << "Physical Pos = \t\t" << p_pos << "\n";
    resetFile << "Virtual Pos = \t\t" << v_pos << "\n";
    resetFile << "Physical Ori = \t\t" << p_ori.toEuler() << "\n";
    resetFile << "Virtual Ori = \t\t" << v_ori.toEuler() << "\n";
    resetFile << "Heading Offset = \t" << headingOffset << "\n";
    resetFile << "Current Seg# = \t\t" << currentSegment << "\n";
    resetFile << "Walked = \t\t" << virtualPathLength << "m\n";
    resetFile << "Best angle = \t\t" << bestAngle << "\n";
    resetFile << "Paths:\n";
    for (unsigned int i = 0; i < paths.size(); ++i) {
        resetFile << "\t" << paths[i].first.toString() << "\n";
    }

    resetFile << std::flush;
    resetFile.close();
}
#endif
