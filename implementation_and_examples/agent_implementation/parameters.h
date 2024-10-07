//
// Created by hugo on 7-10-24.
//


#ifndef IMPLEMENTATION_AND_EXAMPLES_PARAMETERS_H
#define IMPLEMENTATION_AND_EXAMPLES_PARAMETERS_H
//
#define DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
#define CLOSE_SMALL_AREAS
#define SEPARATE_FRONTIERS
#define WALL_FOLLOWING_ENABLED
#define BLACKLIST_FRONTIERS // If this is defined, DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED will automatically be defined
#ifdef BLACKLIST_FRONTIERS
    #ifndef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
        #define DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    #endif
#endif
#define WALKING_STATE_WHEN_NO_FRONTIERS

namespace PiPuckParameters {

    constexpr double DISTANCE_SENSOR_NOISE_CM = 0.0;
    constexpr double ORIENTATION_NOISE_DEGREES = 5.0;
    constexpr double POSITION_NOISE_CM = 5.0;

#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
constexpr double FRONTIER_DIST_UNTIL_REACHED = 1.0;
#endif


    constexpr double PROXIMITY_RANGE = 2.0;

    constexpr double TURN_THRESHOLD_DEGREES = 8.0;

    constexpr double AGENT_ROBOT_DIAMETER = 0.08;

    constexpr double OBJECT_SAFETY_RADIUS = 0.1;
    constexpr double AGENT_SAFETY_RADIUS = AGENT_ROBOT_DIAMETER + 0.1;

    constexpr double VIRTUAL_WALL_AVOIDANCE_WEIGHT = 1.1;
    constexpr double AGENT_COHESION_WEIGHT = 0;//0.23;
    constexpr double AGENT_AVOIDANCE_WEIGHT = 1.15;
    constexpr double AGENT_ALIGNMENT_WEIGHT = 0.5;//0.5;
    constexpr double UNEXPLORED_FRONTIER_WEIGHT = 0.3;

    constexpr double FRONTIER_DISTANCE_WEIGHT = 0.1;//0.001;
    constexpr double FRONTIER_SIZE_WEIGHT = 1.0;

    constexpr double FRONTIER_SEARCH_DIAMETER = 8.0;

    constexpr double AGENT_COHESION_RADIUS = 1.5;
    constexpr double AGENT_AVOIDANCE_RADIUS = 0.68;
    constexpr double AGENT_ALIGNMENT_RADIUS = 1.5;
    constexpr double OBJECT_AVOIDANCE_RADIUS = AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS + 0.2;

    constexpr double RECEIVED_OBSERVATION_ADAPTION_PROBABILITY = 75; //Adaption probability for the agent of adapting received new observations by other agents
    constexpr double OWN_OBSERVATION_ADAPTION_PROBABILITY_FREE = 90; //Adaption probability if known cell is FREE for the agent of adapting received new observations by itself
    constexpr double OWN_OBSERVATION_ADAPTION_PROBABILITY_OCCUPIED = 10; //Adaption probability if known cell is OCCUPIED for the agent of adapting received new observations by itself

    constexpr double BLACKLIST_CHANCE_PER_COUNT = 30;
    constexpr double MIN_ALLOWED_DIST_BETWEEN_FRONTIERS = 1.0;

    constexpr int CLOSEST_COORDINATE_HIT_COUNT_BEFORE_BLACKLIST = 2;
}

#endif //IMPLEMENTATION_AND_EXAMPLES_PARAMETERS_H
