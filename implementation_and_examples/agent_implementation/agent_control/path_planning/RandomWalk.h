
#ifndef IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H
#define IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H

#include "agent_implementation/feature_config.h"
#include <argos3/core/utility/math/vector2.h>
#include "agent_implementation/utils/coordinate.h"

class Agent;

class RandomWalk {
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
public:
    void randomWalk(Agent* agent, argos::CVector2 &targetVector);
    bool randomWalkedFarEnough(Agent* agent);
    bool randomWalking = false;

private:
    const float farEnoughDistance = 0.5;
    Coordinate walkStart;
    int ticksWalkStart = 0;
    int maxTicksSamePosition = 150;

#endif
};

#endif //IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H

