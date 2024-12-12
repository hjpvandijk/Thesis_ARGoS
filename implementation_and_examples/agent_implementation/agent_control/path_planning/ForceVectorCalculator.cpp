#include "ForceVectorCalculator.h"
#include "agent.h"


/** Calculate the vector to avoid the virtual walls
 * If the agent is close to the border, create a vector pointing away from the border
 * Implemented according to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * However, the vector directions are flipped compared to the paper, as the paper uses a different coordinate system
 * @return a vector pointing away from the border
 *
 */
argos::CVector2 ForceVectorCalculator::getVirtualWallAvoidanceVector(Agent* agent) {
    //If the agent is close to the border, create a vector pointing away from the border
    argos::CVector2 virtualWallAvoidanceVector = {0, 0};

    if (agent->position.x < agent->left_right_borders.x) {
        virtualWallAvoidanceVector.SetX(1);
    } else if (agent->left_right_borders.x <= agent->position.x && agent->position.x <= agent->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(0);
    } else if (agent->position.x > agent->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(-1);
    }

    if (agent->position.y < agent->upper_lower_borders.y) {
        virtualWallAvoidanceVector.SetY(1);
    } else if (agent->upper_lower_borders.y <= agent->position.y && agent->position.y <= agent->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(0);
    } else if (agent->position.y > agent->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(-1);
    }


    return virtualWallAvoidanceVector;

}

bool ForceVectorCalculator::getAverageNeighborLocation(Agent* agent, Coordinate *averageNeighborLocation, double range) {
    int nAgentsWithinRange = 0;
    for (const auto &agentLocation: agent->agentLocations) {
        if((agentLocation.second.second - agent->elapsed_ticks) / agent->ticks_per_second > agent->AGENT_LOCATION_RELEVANT_DURATION_S) continue;
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(agentLocation.second.first.x, agentLocation.second.first.y)
                - argos::CVector2(agent->position.x, agent->position.y);

        if (vectorToOtherAgent.Length() < range) {
            averageNeighborLocation->x += agentLocation.second.first.x;
            averageNeighborLocation->y += agentLocation.second.first.y;
            nAgentsWithinRange++;
        }
    }
    //If no agents are within range, there is no average location
    if (nAgentsWithinRange == 0) {
        return false;
    }
    //Else calculate the average position of the agents within range
    averageNeighborLocation->x /= nAgentsWithinRange;
    averageNeighborLocation->y /= nAgentsWithinRange;

    return true;
}

argos::CVector2 ForceVectorCalculator::calculateAgentCohesionVector(Agent* agent) {
    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->AGENT_COHESION_RADIUS);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToOtherAgents;
}


/**
 * Calculate the vector to avoid other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the opposite direction of the average location of these agents.
 * @return a vector pointing away from the average location other agents
 */
argos::CVector2 ForceVectorCalculator::calculateAgentAvoidanceVector(Agent* agent) {

    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->AGENT_AVOIDANCE_RADIUS);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between agent agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToOtherAgents * -1;
}

/**
 * Calculate the vector to align with other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the direction of the average vector of these agents, considering speed.
 * @return
 */
argos::CVector2 ForceVectorCalculator::calculateAgentAlignmentVector(Agent* agent) {
    argos::CVector2 alignmentVector = {0, 0};
    int nAgentsWithinRange = 0;


    //Get the velocities of the agents within range
    for (const auto &agentVelocity: agent->agentVelocities) {
        std::string agentID = agentVelocity.first;
        Coordinate otherAgentLocation = agent->agentLocations[agentID].first;
        argos::CVector2 agentVector = agentVelocity.second.first;
        double agentSpeed = agentVelocity.second.second;
        argos::CVector2 vectorToOtherAgent = argos::CVector2(otherAgentLocation.x, otherAgentLocation.y)
                                             - argos::CVector2(agent->position.x, agent->position.y);
        if (vectorToOtherAgent.Length() < agent->AGENT_ALIGNMENT_RADIUS) {
            alignmentVector += agentVector * agentSpeed;
            nAgentsWithinRange++;
        }
    }

    //If no agents are within range, there is no average velocity
    if (nAgentsWithinRange == 0) {
        return {0, 0};
    }

    //Else calculate the average velocity of the agents within range
    alignmentVector /= nAgentsWithinRange;
    return alignmentVector;
}

// Union-Find (Disjoint-Set) data structure
class UnionFind {
public:
    void add(int x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            rank[x] = 0;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unionSets(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }

private:
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
};

void mergeAdjacentFrontiers(const std::vector<quadtree::Box> &frontiers,
                            std::vector<std::vector<quadtree::Box>> &frontierRegions) {
    UnionFind uf;
    std::unordered_map<int, quadtree::Box> boxMap;

    // Initialize union-find structure
    for (int i = 0; i < frontiers.size(); ++i) {
        uf.add(i);
        boxMap[i] = frontiers[i];
    }

    // Check adjacency and union sets
    for (int i = 0; i < frontiers.size(); ++i) {
        for (int j = i + 1; j < frontiers.size(); ++j) {
            Coordinate centerI = frontiers[i].getCenter();
            Coordinate centerJ = frontiers[j].getCenter();
            double distance = sqrt(pow(centerI.x - centerJ.x, 2) + pow(centerI.y - centerJ.y, 2));
            double threshold = sqrt(2 * pow(frontiers[i].getSize() * 0.5 + frontiers[j].getSize() * 0.5, 2));
            if (distance <= threshold) {
                uf.unionSets(i, j);
            }
        }
    }

    // Group boxes by their root set representative
    std::unordered_map<int, std::vector<quadtree::Box>> regions;
    for (int i = 0; i < frontiers.size(); ++i) {
        int root = uf.find(i);
        regions[root].push_back(frontiers[i]);
    }

    // Convert to the required format
    for (const auto &region: regions) {
        frontierRegions.push_back(region.second);
    }
}

argos::CVector2 ForceVectorCalculator::calculateUnexploredFrontierVector(Agent* agent) {
    //According to Dynamic frontier-led swarming:
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    //F* = arg min (Ψ_D||p-G^f||_2 - Ψ_S * J^f) for all frontiers F^f in F
    //F^f is a frontier, a segment that separates explored cells from unexplored cells.

    //Where Ψ_D is the frontier distance weight
    //p is the agent position
    //G^f is the frontier position defined as G^f = (Sum(F_j^f))/J^f So the sum of the cell locations divided by the amount of cells
    //Ψ_S is the frontier size weight
    //J^f is the number of cells in the frontier F^f

    //A cell is a frontier iff:
    //1. Occupancy = explored
    //2. At least one neighbor is unexplored using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

    //TODO: Need to keep search area small for computation times. Maybe when in range only low scores, expand range or search a box besides.
    std::vector<quadtree::Box> frontiers = agent->quadtree->queryFrontierBoxes(agent->position, agent->FRONTIER_SEARCH_DIAMETER,
                                                                              agent->elapsed_ticks /
                                                                              agent->ticks_per_second);
    agent->current_frontiers = frontiers;

    // Initialize an empty vector of vectors to store frontier regions
    std::vector<std::vector<quadtree::Box>> frontierRegions = {};

    mergeAdjacentFrontiers(frontiers, frontierRegions);

// Iterate over each frontier box to merge adjacent ones into regions
    for (auto frontier: frontiers) {
        bool added = false; // Flag to check if the current frontier has been added to a region

        // Iterate over existing regions to find a suitable one for the current frontier
        for (auto &region: frontierRegions) {
            for (auto box: region) {
                // Calculate the center coordinates of the current box and the frontier
                Coordinate boxCenter = box.getCenter();
                Coordinate frontierCenter = frontier.getCenter();

                // Check if the distance between the box and the frontier is less than or equal to
                // the average diagonal of the two boxes (ensuring adjacency)
                if (sqrt(pow(boxCenter.x - frontierCenter.x, 2) + pow(boxCenter.y - frontierCenter.y, 2)) <=
                    sqrt(2 * pow(frontier.getSize() * 0.5 + box.getSize() * 0.5, 2))) {
                    region.push_back(frontier); // Add the frontier to the current region
                    added = true; // Mark the frontier as added
                    break; // Exit the loop since the frontier has been added to a region
                }
            }
        }

        // If the frontier was not added to any existing region, create a new region with it
        if (!added) {
            frontierRegions.push_back({frontier});
        }
    }

    agent->current_frontier_regions = frontierRegions;

    //Now we have all frontier cells merged into frontier regions
    //Find F* by using the formula above
    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
    //Ψ_S = FRONTIER_SIZE_WEIGHT

    //Initialize variables to store the best frontier region and its score
    std::vector<quadtree::Box> bestFrontierRegion = {};
    Coordinate bestFrontierRegionCenter = {MAXFLOAT, MAXFLOAT};
    double bestFrontierScore = std::numeric_limits<double>::max();
    std::vector<std::pair<Coordinate, Coordinate>> bestRoute = {};

    //Iterate over all frontier regions to find the best one
    for (const auto &region: frontierRegions) {
        //Calculate the average position of the frontier region
        double sumX = 0;
        double sumY = 0;
        double totalNumberOfCellsInRegion = 0;
        for (auto box: region) {
            double cellsInBox = box.getSize() / agent->quadtree->getSmallestBoxSize();
            assert(cellsInBox == 1);
            sumX += box.getCenter().x *
                    cellsInBox; //Take the box size into account (parent nodes will contain the info about all its children)
            sumY += box.getCenter().y * cellsInBox;
            totalNumberOfCellsInRegion += cellsInBox;
        }
        double frontierRegionX = sumX / totalNumberOfCellsInRegion;
        double frontierRegionY = sumY / totalNumberOfCellsInRegion;

#ifdef AVOID_UNREACHABLE_FRONTIERS
        if (agent->frontierEvaluator.skipFrontier(agent, frontierRegionX, frontierRegionY)) continue; //Skip agent frontier
#endif
        argos::CVector2 vectorToFrontier = argos::CVector2(frontierRegionX - agent->position.x, frontierRegionY - agent->position.y).Rotate(-agent->heading);
        std::vector<std::pair<Coordinate, Coordinate>> route_to_frontier = {{agent->position, Coordinate{frontierRegionX, frontierRegionY}}};

        //Calculate the distance between the agent and the frontier region
#ifdef PATH_PLANNING_ENABLED
        //Calculate distance of route
        double distance = 0;
        route_to_frontier = agent->pathPlanner.getRoute(agent, agent->position, {frontierRegionX, frontierRegionY});
        for (auto edge: route_to_frontier) {
            distance += sqrt(pow(edge.first.x - edge.second.x, 2) + pow(edge.first.y - edge.second.y, 2));
        }
#else
        double distance = sqrt(pow(frontierRegionX - agent->position.x, 2) + pow(frontierRegionY - agent->position.y, 2));
#endif
        //Relative vector to heading


        //Only need to compare the motion power usage of the agent to the frontier region
        auto [powerUsage, duration] = agent->batteryManager.estimateMotionPowerUsage(agent, {vectorToFrontier});

        //Calculate the score of the frontier region
        double score =
                agent->FRONTIER_DISTANCE_WEIGHT * distance - agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
                agent->FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->FRONTIER_REACH_DURATION_WEIGHT * duration;



#ifdef SEPARATE_FRONTIERS
        std::vector<double> distancesFromOtherAgents = {};

        for (const auto &agentLocationPair: agent->agentLocations) {
            //Get the distance between the frontier and the last known location of other agents
            Coordinate agentLocation = agentLocationPair.second.first;
            double distanceFromOtherAgent = sqrt(
                    pow(frontierRegionX - agentLocation.x, 2) + pow(frontierRegionY - agentLocation.y, 2));
            distancesFromOtherAgents.push_back(distanceFromOtherAgent);
        }

        //TODO: add battery and duration to the score
        //TODO: Probably better; exchange messages about selected frontiers, so that agents can avoid each other's frontiers (highest ID priority for example)
        //If that frontier is best to visit for a different agent, skip it.
        bool otherAgentLowerScore = false;
        for (auto distanceFromOtherAgent: distancesFromOtherAgents) {
            double otherAgentScore = agent->FRONTIER_DISTANCE_WEIGHT * distanceFromOtherAgent -
                    agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion;
            if (otherAgentScore < score) {
                otherAgentLowerScore = true;
                break;
            }
        }
        if (otherAgentLowerScore) continue;
#endif

        //If the score is lower than the best score, update the best score and best frontier region
        if (score < bestFrontierScore) {
            bestFrontierScore = score;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
            bestRoute = route_to_frontier;
        }
    }


    agent->currentBestFrontier = bestFrontierRegionCenter;
    agent->route_to_best_frontier = bestRoute;



    //Calculate the vector to the best frontier region
    argos::CVector2 vectorToBestFrontier = argos::CVector2(bestFrontierRegionCenter.x, bestFrontierRegionCenter.y)
                                           - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToBestFrontier;
}

/**
 * Calculate the vector to avoid objects:
 * Find the closest relative angle that is free of objects within a certain range.
 * According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * @return The vector towards the free direction
 */
bool ForceVectorCalculator::calculateObjectAvoidanceAngle(Agent* agent, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {

    //Get occupied boxes within range
    std::vector<quadtree::Box> occupiedBoxes = agent->quadtree->queryOccupiedBoxes(agent->position,
                                                                                   agent->OBJECT_AVOIDANCE_RADIUS * 2.0,
                                                                                  agent->elapsed_ticks /
                                                                                  agent->ticks_per_second);

    double angleInterval = argos::CDegrees(360 / agent->ANGLE_INTERVAL_STEPS).GetValue();

    //Create set of free angles ordered to be used for wall following
    std::set<argos::CDegrees, CustomComparator> freeAngles(
            CustomComparator(agent->wallFollower.wallFollowingDirection, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue()));

//Add free angles from -180 to 180 degrees
    for (int a = 0; a < agent->ANGLE_INTERVAL_STEPS; a++) {
        auto angle = argos::CDegrees(a * 360 / agent->ANGLE_INTERVAL_STEPS - 180);
        freeAngles.insert(angle);
    }


    //For each occupied box, find the angles that are blocked relative to the agent
    for (auto box: occupiedBoxes) {


        argos::CVector2 OC = argos::CVector2(box.getCenter().x - agent->position.x,
                                             box.getCenter().y - agent->position.y);
        argos::CRadians Bq = argos::ASin(
                std::min(agent->AGENT_SAFETY_RADIUS + agent->OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
        argos::CRadians Eta_q = OC.Angle();
        if (agent->AGENT_SAFETY_RADIUS + agent->OBJECT_SAFETY_RADIUS > OC.Length())
            argos::LOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << agent->AGENT_SAFETY_RADIUS
                           << " + " << agent->OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;

        argos::CDegrees minAngle = ToDegrees((Eta_q - Bq).SignedNormalize());
        argos::CDegrees maxAngle = ToDegrees((Eta_q + Bq).SignedNormalize());

        if (maxAngle < minAngle) {
            argos::CDegrees temp = minAngle;
            minAngle = maxAngle;
            maxAngle = temp;
        }


        //Round to multiples of 360/ANGLE_INTERVAL_STEPS
        double minAngleVal = minAngle.GetValue();
        double intervalDirectionMin = (minAngleVal < 0) ? -angleInterval : angleInterval;
//
        auto minRoundedAngle1 = (int) (minAngleVal / angleInterval) * angleInterval;
        auto minRoundedAngle2 = minRoundedAngle1 + intervalDirectionMin;

        auto roundedMinAngle = argos::CDegrees(
                (abs(minAngleVal - minRoundedAngle1) >= abs(minRoundedAngle2 - minAngleVal)) ? minRoundedAngle2
                                                                                             : minRoundedAngle1);

        double maxAngleVal = maxAngle.GetValue();
        double intervalDirectionMax = (maxAngleVal < 0) ? -angleInterval : angleInterval;

        auto maxRoundedAngle1 = (int) (maxAngleVal / angleInterval) * angleInterval;
        auto maxRoundedAngle2 = maxRoundedAngle1 + intervalDirectionMax;

        auto roundedMaxAngle = argos::CDegrees(
                (abs(maxAngleVal - maxRoundedAngle1) >= abs(maxRoundedAngle2 - maxAngleVal)) ? maxRoundedAngle2
                                                                                             : maxRoundedAngle1);

        //Block all angles within range
        for (int a = 0; a < agent->ANGLE_INTERVAL_STEPS; a++) {
            auto angle = argos::CDegrees(a * 360 / agent->ANGLE_INTERVAL_STEPS - 180);

            if (NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) <= argos::CRadians(0) &&
                NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) >= argos::CRadians(0)) {
                if (angle >= roundedMinAngle && angle <= roundedMaxAngle) {
                    freeAngles.erase(angle);
                }

            } else if (NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) >= argos::CRadians(0) &&
                       NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) <= argos::CRadians(0)) {

                if (angle <= roundedMinAngle || angle >= roundedMaxAngle) {
                    freeAngles.erase(angle);
                }
            } else {
                argos::LOGERR << "Error: Eta_q not within range" << std::endl;
            }
        }

    }

    agent->freeAnglesVisualization.clear();
    for (auto freeAngle: freeAngles) {
        agent->freeAnglesVisualization.insert(freeAngle);
    }
    if (freeAngles.empty()) return false;


    //Get free angle closest to heading
    auto closestFreeAngle = *freeAngles.begin();
#ifdef WALL_FOLLOWING_ENABLED
    if (agent->wallFollower.wallFollowingDirection != 0) { //If wall following is not 0, find the closest free angle to the target angle.
        for (auto freeAngle: freeAngles) {
            if (std::abs(NormalizedDifference(ToRadians(freeAngle), targetAngle).GetValue()) <
                std::abs(NormalizedDifference(ToRadians(closestFreeAngle), targetAngle).GetValue())) {
                closestFreeAngle = freeAngle;
            }
        }
    }
#endif


    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);

#ifdef WALL_FOLLOWING_ENABLED//If wall following is enabled
    agent->wallFollower.wallFollowing(agent, freeAngles, &closestFreeAngle, &closestFreeAngleRadians, relativeObjectAvoidanceAngle, targetAngle);
#endif
#ifdef PATH_PLANNING_ENABLED
    agent->pathFollower.followPath(agent, freeAngles, relativeObjectAvoidanceAngle, targetAngle);
#endif
    return true;

}
