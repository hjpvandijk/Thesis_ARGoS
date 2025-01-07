//
// Created by hugo on 7-1-25.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H
#define IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H

#include <vector>
#include "agent_implementation/utils/coordinate.h"

class Agent;


class Algorithms {
public:
    static std::vector<Coordinate> bresenhamLine(Agent *agent, Coordinate coordinate1, Coordinate coordinate2);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H
