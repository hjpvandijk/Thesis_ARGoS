#ifndef IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H
#define IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H

#include <set>
#include "agent_implementation/utils/coordinate.h"
#include "agent_implementation/utils/CustomComparator.h"

class Agent;


class PathFollower {
public:
//    void WallFollower::wallFollowing(Agent* agent, std::set<argos::CDegrees, CustomComparator>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {
    void followPath(Agent *agent, std::set<argos::CDegrees, CustomComparator>& freeAngles, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle);

private:
    int current_path_section = 0;
    std::vector<std::pair<Coordinate, Coordinate>> current_route;
};


#endif //IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H
