#include "FrontierEvaluator.h"
#include "agent.h"

FrontierEvaluator::FrontierEvaluator(int closest_coordinate_hit_count_before_decreasing_confidence,
                                     int max_ticks_in_hitpoint) {
    this->closest_coordinate_hit_count_before_decreasing_confidence = closest_coordinate_hit_count_before_decreasing_confidence;
    this->max_ticks_in_hitpoint = max_ticks_in_hitpoint;
}


/**
 * Decide if the frontier should be skipped due to being low confidence.
 * This is dependent the confidence of the frontier and a random chance, or if we were already avoiding it.
 * @param frontierRegionX
 * @param frontierRegionY
 * @return True if the frontier should be skipped, false otherwise
 */
bool FrontierEvaluator::skipFrontier(Agent* agent, double frontierRegionX, double frontierRegionY) {
    bool alreadyAvoiding = (std::find(this->avoidingFrontiers.begin(), this->avoidingFrontiers.end(),
                                      Coordinate{frontierRegionX, frontierRegionY}) != this->avoidingFrontiers.end());
    if (alreadyAvoiding) return true;

    float Lconfidence = agent->quadtree->getConfidenceFromCoordinate({frontierRegionX, frontierRegionY});
    float Pconfidence = std::exp(Lconfidence) / (1 + std::exp(Lconfidence)); // P(n|z1:t) = exp(L(P(n|z1:t))) / (1 + exp(L(P(n|z1:t))))
    int randomChance = rand() % 100;
    //P < 0.5 = occupied, P > 0.5 = free
    if (Pconfidence < agent->P_FREE_THRESHOLD) { //If the frontier might be occupied
        float Pfree = (Pconfidence - agent->P_OCCUPIED_THRESHOLD)  / (1-agent->P_OCCUPIED_THRESHOLD); // Rescale to 0-1

        if (randomChance > Pfree * 100.0) { //The higher the confidence, the lower the chance to skip
            this->avoidingFrontiers.push_back({frontierRegionX, frontierRegionY}); //Currently avoiding said frontier
            return true;
        }
    }
    return false;
}


/**
 * Reset the frontiers avoiding list if the agent is close to a target frontier.
 * This gives the agent the chance to select all frontiers (even with low confidence) again.
 * @param unexploredFrontierVector
 */
void FrontierEvaluator::resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector) {
    //If the agent is close to the frontier, reset all frontiers avoiding flags (we're giving them another chance)
#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    if (unexploredFrontierVector.Length() <= agent->FRONTIER_DIST_UNTIL_REACHED) {
        this->avoidingFrontiers.clear();
    }
#endif
}

/**
 * Check if the target frontier has low confidence.
 * @return
 */
bool FrontierEvaluator::frontierHasLowConfidenceOrAvoiding(Agent* agent){
    if(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) return false;
    if(std::find(this->avoidingFrontiers.begin(), this->avoidingFrontiers.end(), agent->currentBestFrontier) != this->avoidingFrontiers.end()) return true;
    float LConfidence = agent->quadtree->getConfidenceFromCoordinate(agent->currentBestFrontier);
    float PConfidence = std::exp(LConfidence) / (1 + std::exp(LConfidence)); // P(n|z1:t) = exp(L(P(n|z1:t))) / (1 + exp(L(P(n|z1:t))))
    if (PConfidence >= agent->P_FREE_THRESHOLD) {
        return false;
    }

    return true;
}

/**
 * Update the confidence of cells if they are around a currently unreachable frontier.
 * If the agent is hitting the same hitpoint multiple times, decrease the frontier confidence.
 */
void FrontierEvaluator::updateConfidenceIfFrontierUnreachable(Agent* agent) {
    //Increase the reachability confidence of all cells closeby
//    quadtree->updateConfidenceIfFrontierUnreachable(this->position, MIN_ALLOWED_DIST_BETWEEN_FRONTIERS,
//                               P_POSITION, this->elapsed_ticks / this->ticks_per_second);


    Coordinate target = agent->currentBestFrontier;
//If we have no frontier (walking state), calculate the distance to the subtarget instead
#ifdef PATH_PLANNING_ENABLED
    if (!(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT})) target = agent->subTarget;
#else
    if (agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) target = agent->subTarget;
#endif
    double distanceToTarget = sqrt(pow(agent->currentBestFrontier.x - agent->position.x, 2) +
                                   pow(agent->currentBestFrontier.y - agent->position.y, 2));

    double distanceToClosestPoint = sqrt(pow(agent->position.x - this->closestCoordinateToCurrentFrontier.x, 2) +
                                         pow(agent->position.y - this->closestCoordinateToCurrentFrontier.y, 2));

    if (!(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) && agent->currentBestFrontier == agent->previousBestFrontier || !(agent->subTarget == Coordinate{MAXFLOAT,
                                                                                                                                                                     MAXFLOAT})) { //If we are still on route to the same frontier, or to a subtarget
//Check if the distance to the frontier has decreased in the last timeToCheckFrontierDistS seconds
        if (distanceToTarget + 0.05 < this->minDistFromFrontier) { //If the distance has decreased by at least 5cm.
            this->minDistFromFrontier = distanceToTarget;
            this->closestCoordinateCounter = 0;
            this->closestCoordinateToCurrentFrontier = agent->position;
            this->lastTickInFrontierHitPoint = false;
            this->ticksInHitpoint = 0;
        } else if (distanceToClosestPoint <=
                   0.5*agent->OBJECT_AVOIDANCE_RADIUS) { //If we are again on the closest point to the frontier
            this->ticksInHitpoint++;
            if (!this->lastTickInFrontierHitPoint || this->ticksInHitpoint >= this->max_ticks_in_hitpoint) {
                this->closestCoordinateCounter++; //Increase the counter
                if (this->closestCoordinateCounter >= this->closest_coordinate_hit_count_before_decreasing_confidence || this->ticksInHitpoint >= this->max_ticks_in_hitpoint) { //If we have hit closest point  too often (we are in a loop)
                    //Decrease the confidence of all cells closeby
                    agent->quadtree->updateConfidence(target, agent->MIN_ALLOWED_DIST_BETWEEN_FRONTIERS,
                                                      agent->P_AVOIDANCE, agent->elapsed_ticks / agent->ticks_per_second);
                    this->avoidingFrontiers.push_back(target);
                    this->closestCoordinateCounter = 0; // Reset counter
                }
                this->lastTickInFrontierHitPoint = true;
                this->ticksInHitpoint = 0;
            }
        } else { //If we are not on the closest point to the frontier, set the flag to false
            this->lastTickInFrontierHitPoint = false;
            this->ticksInHitpoint = 0;
        }


    } else { //If we are not on route to the same frontier, set the min distance and time
        this->minDistFromFrontier = MAXFLOAT;
        this->closestCoordinateCounter = 0;
        this->closestCoordinateToCurrentFrontier = Coordinate{MAXFLOAT, MAXFLOAT};
        this->lastTickInFrontierHitPoint = false;
        this->ticksInHitpoint = 0;


//        this->timeFrontierDistDecreased = this->elapsed_ticks / this->ticks_per_second;
    }
}