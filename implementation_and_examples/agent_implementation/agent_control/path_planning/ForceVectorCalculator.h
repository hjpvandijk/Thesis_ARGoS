#ifndef IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H


#include <argos3/core/utility/math/vector2.h>
#include "agent_implementation/utils/coordinate.h"
class Agent;

class ForceVectorCalculator {
public:
    ForceVectorCalculator() = default;

    static argos::CVector2 getVirtualWallAvoidanceVector(Agent *agent) ;

    static bool getAverageNeighborLocation(Agent *agent, Coordinate *averageNeighborLocation, double range);

    static argos::CVector2 calculateAgentCohesionVector(Agent *agent);

    static argos::CVector2 calculateAgentAvoidanceVector(Agent *agent);

    static argos::CVector2 calculateAgentAlignmentVector(Agent *agent);

    static argos::CVector2 calculateUnexploredFrontierVector(Agent *agent);

    static bool calculateObjectAvoidanceAngle(Agent* agent, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle, bool frontier_vector_zero);

};
#endif //IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H
