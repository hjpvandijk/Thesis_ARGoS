#ifndef IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H
#define IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H

#include "agent_implementation/utils/CustomComparator.h"
#include "agent_implementation/utils/coordinate.h"

class Agent;

class WallFollower {
public:
    int wallFollowingDirection = 0;
    Coordinate wallFollowingSubTarget = {MAXFLOAT, MAXFLOAT};
    int prevWallFollowingDirection = 0;
    Coordinate wallFollowingHitPoint = {MAXFLOAT, MAXFLOAT};

    WallFollower() = default;
    void wallFollowing(Agent* agent, std::set<argos::CDegrees, CustomComparator>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle);

};

#endif //IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H