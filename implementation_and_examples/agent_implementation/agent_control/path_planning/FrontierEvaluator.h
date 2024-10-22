#ifndef IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H


#include <argos3/core/utility/math/vector2.h>
#include "agent_implementation/utils/coordinate.h"

class Agent;

class FrontierEvaluator {
public:
    std::vector<Coordinate> avoidingFrontiers;
    double minDistFromFrontier = MAXFLOAT;
    Coordinate closestCoordinateToCurrentFrontier = {MAXFLOAT, MAXFLOAT};
    int closestCoordinateCounter = 0;
    int ticksInHitpoint = 0;
    int closest_coordinate_hit_count_before_decreasing_confidence;
    int max_ticks_in_hitpoint;
    bool lastTickInFrontierHitPoint = false;

    FrontierEvaluator() = default;
    FrontierEvaluator(int closest_coordinate_hit_count_before_decreasing_confidence, int max_ticks_in_hitpoint);

    bool skipFrontier(Agent* agent, double frontierRegionX, double frontierRegionY);
    void resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector);
    bool frontierHasLowConfidenceOrAvoiding(Agent* agent);
    void updateConfidenceIfFrontierUnreachable(Agent* agent);
    };


#endif //IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
