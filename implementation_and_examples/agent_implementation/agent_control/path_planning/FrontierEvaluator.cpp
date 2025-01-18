#include "FrontierEvaluator.h"
#include "agent.h"



/**
 * Reset the frontiers avoiding list if the agent is close to a target frontier.
 * This gives the agent the chance to select all frontiers (even with low confidence) again.
 * @param unexploredFrontierVector
 */
void FrontierEvaluator::resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector) {
    //If the agent is close to the frontier, reset all frontiers avoiding flags (we're giving them another chance)
#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    if (unexploredFrontierVector.Length() <= agent->config.FRONTIER_DIST_UNTIL_REACHED) {
        this->avoidingFrontiers.clear();
    }
#endif
}

/**
 * Check if the target frontier has low confidence.
 * @return
 */
bool FrontierEvaluator::avoidingFrontier(Agent* agent){
    if(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) return false;
    //If the agent is close to a frontier we are currently avoiding
    for (auto frontier: this->avoidingFrontiers) {
        if (sqrt(pow(frontier.x - agent->currentBestFrontier.x, 2) + pow(frontier.y - agent->currentBestFrontier.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD) {
            return true;
        }
    }
    return false;
}

#ifdef SKIP_UNREACHABLE_FRONTIERS
/**
 * Update the confidence of cells if they are around a currently unreachable frontier.
 * If the agent is hitting the same hitpoint multiple times, decrease the frontier confidence.
 */
void FrontierEvaluator::skipIfFrontierUnreachable(Agent* agent, argos::CRadians objectAvoidanceAngle, argos::CVector2 total_vector) {

    Coordinate target = agent->currentBestFrontier;
//If we have no frontier (walking state), calculate the distance to the subtarget instead
#ifdef PATH_PLANNING_ENABLED
    if (!(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT})) target = agent->subTarget;
#else
    if (agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) target = agent->subTarget;
#endif

    if (!(target == Coordinate{MAXFLOAT, MAXFLOAT}) &&
    sqrt(pow(target.x - previousTarget.x, 2) + pow(target.y - previousTarget.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD/2) { //If we are still on route to almost the same target
        if (std::abs(ToDegrees(objectAvoidanceAngle).GetValue()) > 89) { //If we cannot move within 90 degrees towards the target
            this->countNoDirectionToTarget++;
            if (this->countNoDirectionToTarget >= this->MAX_COUNT_NO_DIRECTION) {
                this->avoidingFrontiers.push_back(target);
                this->countNoDirectionToTarget = 0;
            }
        }

    } else { //If we are not on route to the same frontier, set the min distance and time
        this->countNoDirectionToTarget = 0;


    }
    previousTarget = target;

}
#endif