//
// Created by hugo on 22-1-25.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H
#define IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H


#include <argos3/core/utility/math/vector2.h>
#include "agent_implementation/utils/coordinate.h"

class Agent;

class RandomWalk {
public:
    void randomWalk(Agent* agent, argos::CVector2 &targetVector);
    bool randomWalkedFarEnough(Agent* agent);
    bool randomWalking = false;

private:
    const float farEnoughDistance = 0.5;
    Coordinate walkStart;

};


#endif //IMPLEMENTATION_AND_EXAMPLES_RANDOMWALK_H
