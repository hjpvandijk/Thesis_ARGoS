#ifndef IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H


#include <argos3/core/utility/math/vector2.h>
#include "agent_implementation/utils/coordinate.h"

class Agent;

class FrontierEvaluator {
public:
    std::vector<Coordinate> avoidingFrontiers;

    //If the agent is counts this many ticks without a direction to the target, it will skip the target
    int MAX_COUNT_NO_DIRECTION = 150;

    //Count the number of ticks the agent has no direction to the target
    int countNoDirectionToTarget = 0;


    FrontierEvaluator() = default;
    FrontierEvaluator(int closest_coordinate_hit_count_before_decreasing_confidence, int max_ticks_no_direction);

    void resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector);
    bool avoidingFrontier(Agent* agent);
    void skipIfFrontierUnreachable(Agent* agent, argos::CRadians objectAvoidanceAngle, argos::CVector2 total_vector);

private:
    Coordinate previousTarget = {MAXFLOAT, MAXFLOAT};
    };


#endif //IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
