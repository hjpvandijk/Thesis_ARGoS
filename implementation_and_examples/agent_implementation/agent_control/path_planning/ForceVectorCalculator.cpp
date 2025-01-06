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
        if((std::get<2>(agentLocation.second) - agent->elapsed_ticks) / agent->ticks_per_second > agent->AGENT_LOCATION_RELEVANT_DURATION_S) continue;
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(std::get<0>(agentLocation.second) .x, std::get<0>(agentLocation.second).y)
                - argos::CVector2(agent->position.x, agent->position.y);

        if (vectorToOtherAgent.Length() < range) {
            averageNeighborLocation->x += std::get<0>(agentLocation.second) .x;
            averageNeighborLocation->y += std::get<0>(agentLocation.second).y;
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
        Coordinate otherAgentLocation = std::get<0>(agent->agentLocations[agentID]);
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

argos::CVector2
ForceVectorCalculator::calculateTotalVector(Agent* agent, vectors vectors) {
    vectors.virtualWallAvoidanceVector = agent->VIRTUAL_WALL_AVOIDANCE_WEIGHT * vectors.virtualWallAvoidanceVector;
    vectors.agentCohesionVector = agent->AGENT_COHESION_WEIGHT * vectors.agentCohesionVector; //Normalize first
    vectors.agentAvoidanceVector = agent->AGENT_AVOIDANCE_WEIGHT * vectors.agentAvoidanceVector;
    vectors.agentAlignmentVector = agent->AGENT_ALIGNMENT_WEIGHT * vectors.agentAlignmentVector;
    vectors.targetVector = agent->UNEXPLORED_FRONTIER_WEIGHT * vectors.targetVector;

    //According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    argos::CVector2 total_vector = vectors.previous_total_vector +
                                   vectors.virtualWallAvoidanceVector +
                                   vectors.agentCohesionVector +
                                   vectors.agentAvoidanceVector +
                                   vectors.agentAlignmentVector +
                                   vectors.targetVector;
    if (total_vector.Length() != 0) total_vector.Normalize();

    return total_vector;
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

void mergeAdjacentFrontiers(const std::vector<std::pair<quadtree::Box, double>> &frontiers,
                            std::vector<std::vector<std::pair<quadtree::Box, double>>> &frontierRegions) {
    UnionFind uf;
    std::unordered_map<int, std::pair<quadtree::Box, double>> boxMap;

    // Initialize union-find structure
    for (int i = 0; i < frontiers.size(); ++i) {
        uf.add(i);
        boxMap[i] = frontiers[i];
    }

    // Check adjacency and union sets
    for (int i = 0; i < frontiers.size(); ++i) {
        auto boxI = frontiers[i].first;
        for (int j = i + 1; j < frontiers.size(); ++j) {
            auto boxJ = frontiers[j].first;
            Coordinate centerI = boxI.getCenter();
            Coordinate centerJ = boxJ.getCenter();
            double distance = sqrt(pow(centerI.x - centerJ.x, 2) + pow(centerI.y - centerJ.y, 2));
            double threshold = sqrt(2 * pow(boxI.getSize() * 0.5 + boxJ.getSize() * 0.5, 2));
            if (distance <= threshold) {
                uf.unionSets(i, j);
            }
        }
    }

    // Group boxes by their root set representative
    std::unordered_map<int, std::vector<std::pair<quadtree::Box, double>>> regions;
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
    std::vector<std::pair<quadtree::Box, double>> frontiers = agent->quadtree->queryFrontierBoxes(agent->position, agent->FRONTIER_SEARCH_DIAMETER,
                                                                              agent->elapsed_ticks /
                                                                              agent->ticks_per_second);
    agent->current_frontiers = frontiers;

    // Initialize an empty vector of vectors to store frontier regions
    std::vector<std::vector<std::pair<quadtree::Box, double>>> frontierRegions = {};

    mergeAdjacentFrontiers(frontiers, frontierRegions);

//// Iterate over each frontier box to merge adjacent ones into regions
//    for (auto [frontierbox, frontierpheromone]: frontiers) {
//        bool added = false; // Flag to check if the current frontier has been added to a region
//
//        // Iterate over existing regions to find a suitable one for the current frontier
//        for (auto &region: frontierRegions) {
//            for (auto [box, pheromone]: region) {
//                // Calculate the center coordinates of the current box and the frontier
//                Coordinate boxCenter = box.getCenter();
//                Coordinate frontierCenter = frontierbox.getCenter();
//
//                // Check if the distance between the box and the frontier is less than or equal to
//                // the average diagonal of the two boxes (ensuring adjacency)
//                if (sqrt(pow(boxCenter.x - frontierCenter.x, 2) + pow(boxCenter.y - frontierCenter.y, 2)) <=
//                    sqrt(2 * pow(frontierbox.getSize() * 0.5 + box.getSize() * 0.5, 2))) {
//                    region.push_back({frontierbox, frontierpheromone}); // Add the frontier to the current region
//                    added = true; // Mark the frontier as added
//                    break; // Exit the loop since the frontier has been added to a region
//                }
//            }
//        }
//
//        // If the frontier was not added to any existing region, create a new region with it
//        if (!added) {
//            frontierRegions.push_back({{frontierbox, frontierpheromone}});
//        }
//    }
    //Add the frontier region from the last best frontier to the options, if it is now out of range. To prevent unneccesary switching
    //If we haven't reached the frontier yet, add the region to the list of frontier regions
    if (sqrt(pow(agent->currentBestFrontier.x - agent->position.x, 2) + pow(agent->currentBestFrontier.y - agent->position.y, 2)) > agent->FRONTIER_DIST_UNTIL_REACHED) {
        if (!agent->bestFrontierRegionBoxes.empty()) frontierRegions.push_back(agent->bestFrontierRegionBoxes);
    }

    agent->current_frontier_regions = frontierRegions;
    argos::LOG << "[" << agent->id << "] Found " << frontierRegions.size() << " frontier regions" << std::endl;

    //Now we have all frontier cells merged into frontier regions
    //Find F* by using the formula above
    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
    //Ψ_S = FRONTIER_SIZE_WEIGHT

    //Initialize variables to store the best frontier region and its score
    std::vector<quadtree::Box> bestFrontierRegion = {};
    Coordinate bestFrontierRegionCenter = {MAXFLOAT, MAXFLOAT};
    double highestFrontierFitness = -std::numeric_limits<double>::max();
    std::vector<std::pair<Coordinate, Coordinate>> bestRoute = {};
#ifdef PATH_PLANNING_ENABLED
    int bestFrontierRouteWallFollowingDirection;
#endif

    //Iterate over all frontier regions to find the best one
    for (const auto &region: frontierRegions) {
        //Calculate the average position of the frontier region
        double sumX = 0;
        double sumY = 0;
        double totalNumberOfCellsInRegion = 0;
        double averagePheromoneCertainty = 0;
        for (auto [box, pheromone]: region) {
            double cellsInBox = box.getSize() / agent->quadtree->getSmallestBoxSize();
            assert(cellsInBox == 1);
            sumX += box.getCenter().x *
                    cellsInBox; //Take the box size into account (parent nodes will contain the info about all its children)
            sumY += box.getCenter().y * cellsInBox;
            totalNumberOfCellsInRegion += cellsInBox;
            averagePheromoneCertainty += std::abs(pheromone - 0.5);
        }
        double frontierRegionX = sumX / totalNumberOfCellsInRegion;
        double frontierRegionY = sumY / totalNumberOfCellsInRegion;
        averagePheromoneCertainty /= totalNumberOfCellsInRegion;


#ifdef AVOID_UNREACHABLE_FRONTIERS
        if (agent->frontierEvaluator.skipFrontier(agent, frontierRegionX, frontierRegionY)) continue; //Skip agent frontier
#endif

#ifdef SEPARATE_FRONTIERS
        bool skipFrontier = false;
        for (auto agentLocationTuple: agent->agentLocations) {
            if((std::get<2>(agentLocationTuple.second) - agent->elapsed_ticks) / agent->ticks_per_second > agent->AGENT_LOCATION_RELEVANT_DURATION_S) continue;

            double distanceFromOtherAgentsFrontier = sqrt(pow(frontierRegionX - std::get<1>(agentLocationTuple.second).x, 2) +
                                                          pow(frontierRegionY - std::get<1>(agentLocationTuple.second).y, 2));

            double distanceFromOtherAgentsPosition = sqrt(pow(frontierRegionX - std::get<0>(agentLocationTuple.second).x, 2) +
                                                          pow(frontierRegionY - std::get<0>(agentLocationTuple.second).y, 2));

            if (distanceFromOtherAgentsFrontier <= agent->FRONTIER_CLOSE_DISTANCE) {
                //If the frontier is close to another agent's frontier, the agent with the highest ID has priority
                if (agentLocationTuple.first > agent->id) {
                    //If the other agent has a higher ID, so we can't select this frontier
                    skipFrontier = true;
                    argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
                               << " because it is close to another agent's frontier" << std::endl;
                    argos::LOG << "Distance from other agent's frontier: " << distanceFromOtherAgentsFrontier << std::endl;
                    break;
                }
            }
            if (distanceFromOtherAgentsPosition <= agent->FRONTIER_CLOSE_DISTANCE) {
                //If the frontier is close to another agent's frontier, the agent with the highest ID has priority
                if (agentLocationTuple.first > agent->id) {
                    //If the other agent has a higher ID, so we can't select this frontier
                    skipFrontier = true;
                    argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
                               << " because it is close to another agent's position" << std::endl;
                    argos::LOG << "Distance from other agent's position: " << distanceFromOtherAgentsPosition
                               << std::endl;
                    break;
                }
            }
        }
        if (skipFrontier) continue;
#endif

        argos::CVector2 vectorToFrontier = argos::CVector2(frontierRegionX - agent->position.x, frontierRegionY - agent->position.y).Rotate(-agent->heading);
        std::vector<std::pair<Coordinate, Coordinate>> route_to_frontier = {{agent->position, Coordinate{frontierRegionX, frontierRegionY}}};

        //Calculate the distance between the agent and the frontier region
#ifdef PATH_PLANNING_ENABLED
        //Calculate distance of route
        double distance = 0;
        route_to_frontier.clear();
        int wall_following_direction = agent->pathPlanner.getRoute(agent, agent->position, {frontierRegionX, frontierRegionY}, route_to_frontier);
        if (route_to_frontier.empty()) continue; //If no route is found, skip this frontier
        std::vector<argos::CVector2> relativeRoute = agent->pathPlanner.coordinateRouteToRelativeVectors(route_to_frontier, agent->heading);
        for (auto edge: relativeRoute) {
            distance += edge.Length();
        }
#else
        double distance = sqrt(pow(frontierRegionX - agent->position.x, 2) + pow(frontierRegionY - agent->position.y, 2));
        std::vector<argos::CVector2> relativeRoute = {vectorToFrontier.Rotate(-agent->heading)};
#endif
        //Relative vector to heading

#ifdef BATTERY_MANAGEMENT_ENABLED
        //Only need to compare the motion power usage of the agent to the frontier region
        auto [powerUsage, duration] = agent->batteryManager.estimateMotionPowerUsage(agent, relativeRoute);

//        //Calculate the cost of the frontier region
//        double cost =
//                agent->FRONTIER_DISTANCE_WEIGHT * distance - agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
//                agent->FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->FRONTIER_REACH_DURATION_WEIGHT * duration -
//                agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;

        //Calculate the fitness of the frontier region
        double fitness =
                -agent->FRONTIER_DISTANCE_WEIGHT * distance + agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
                agent->FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->FRONTIER_REACH_DURATION_WEIGHT * duration -
                agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;

//        if (agent->id == "pipuck1") {
//            argos::LOG << "Frontier region: " << frontierRegionX << ", " << frontierRegionY << " Score: " << fitness
//                       << std::endl;
//            argos::LOG << "Distance * weight = -" << agent->FRONTIER_DISTANCE_WEIGHT << " * " << distance << " = -"
//                       << agent->FRONTIER_DISTANCE_WEIGHT * distance << std::endl;
//            argos::LOG << "Size * weight = +" << agent->FRONTIER_SIZE_WEIGHT << " * " << totalNumberOfCellsInRegion
//                       << " = +" << agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion << std::endl;
//            argos::LOG << "Reach battery * weight = -" << agent->FRONTIER_REACH_BATTERY_WEIGHT << " * " << powerUsage
//                       << " = -" << agent->FRONTIER_REACH_BATTERY_WEIGHT * powerUsage << std::endl;
//            argos::LOG << "Reach duration * weight =-" << agent->FRONTIER_REACH_DURATION_WEIGHT << " * " << duration
//                       << " = -" << -agent->FRONTIER_REACH_DURATION_WEIGHT * duration << std::endl;
//            argos::LOG << "Pheromone * weight = -" << agent->FRONTIER_PHEROMONE_WEIGHT << " * "
//                       << averagePheromoneCertainty << " = "
//                       << -agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty << std::endl;
//        }
#else
        //Calculate the cost of the frontier region
        double fitness = -agent->FRONTIER_DISTANCE_WEIGHT * distance + agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
                       agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;
#endif




        //If the cost is lower than the best cost, update the best cost and best frontier region
        if (fitness > highestFrontierFitness) {
            highestFrontierFitness = fitness;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
            agent->bestFrontierRegionBoxes = region;
            bestRoute = route_to_frontier;
#ifdef PATH_PLANNING_ENABLED
            bestFrontierRouteWallFollowingDirection = wall_following_direction;
#endif
        }
    }


    agent->currentBestFrontier = bestFrontierRegionCenter;
    agent->route_to_best_frontier = bestRoute;
#ifdef PATH_PLANNING_ENABLED
    agent->pathPlanner.setTarget(bestFrontierRegionCenter);
    agent->pathPlanner.setWallFollowingDirection(bestFrontierRouteWallFollowingDirection);
#endif



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
bool ForceVectorCalculator::calculateObjectAvoidanceAngle(Agent* agent, argos::CRadians *relativeObjectAvoidanceAngle, ForceVectorCalculator::vectors vectors, argos::CVector2 & total_vector, bool frontier_vector_zero) {

    total_vector = ForceVectorCalculator::calculateTotalVector(agent, vectors);

    argos::CRadians targetAngle = total_vector.Angle();

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

    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);

//    if (frontier_vector_zero) return true; //If the frontier vector is zero, we must follow the force vector, so can't do wall/path following
#ifdef WALL_FOLLOWING_ENABLED//If wall following is enabled
    agent->wallFollower.wallFollowing(agent, vectors, total_vector, freeAngles, &closestFreeAngle, &closestFreeAngleRadians, relativeObjectAvoidanceAngle, targetAngle);
#endif
    return true;

}

/**
 * Check if we need to avoid an agent, and adjust the target vector accordingly
 * Then normalize all agents
 * @param agent
 * @param vectors
 */
void ForceVectorCalculator::checkAvoidAndNormalizeVectors(ForceVectorCalculator::vectors &vectors) {
    //If there are agents to avoid, do not explore
    if (vectors.agentAvoidanceVector.Length() != 0) vectors.targetVector = {0, 0};

    //Normalize vectors if they are not zero
    if (vectors.virtualWallAvoidanceVector.Length() != 0) vectors.virtualWallAvoidanceVector.Normalize();
    if (vectors.agentCohesionVector.Length() != 0) vectors.agentCohesionVector.Normalize();
    if (vectors.agentAvoidanceVector.Length() != 0) vectors.agentAvoidanceVector.Normalize();
    if (vectors.agentAlignmentVector.Length() != 0) vectors.agentAlignmentVector.Normalize();
    if (vectors.targetVector.Length() != 0) vectors.targetVector.Normalize();

}
