#include "ForceVectorCalculatorMatrix.h"
#include "agent.h"
#include "utils/CustomComparator.h"


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
        auto agenttime = std::get<1>(agentLocation.second);
        auto diff = (agent->elapsed_ticks - agenttime );
        auto diffSeconds = diff / agent->ticks_per_second;
        if( (agent->elapsed_ticks - std::get<1>(agentLocation.second)) / agent->ticks_per_second > agent->config.AGENT_LOCATION_RELEVANT_S) continue;
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

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->config.AGENT_COHESION_RADIUS);
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

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->config.AGENT_AVOIDANCE_RADIUS);
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
        if (vectorToOtherAgent.Length() < agent->config.AGENT_ALIGNMENT_RADIUS) {
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
    vectors.virtualWallAvoidanceVector = agent->config.VIRTUAL_WALL_AVOIDANCE_WEIGHT * vectors.virtualWallAvoidanceVector;
    vectors.agentCohesionVector = agent->config.AGENT_COHESION_WEIGHT * vectors.agentCohesionVector; //Normalize first
    vectors.agentAvoidanceVector = agent->config.AGENT_AVOIDANCE_WEIGHT * vectors.agentAvoidanceVector;
    vectors.agentAlignmentVector = agent->config.AGENT_ALIGNMENT_WEIGHT * vectors.agentAlignmentVector;
    vectors.targetVector = agent->config.TARGET_WEIGHT * vectors.targetVector;

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
                            std::vector<std::vector<std::pair<quadtree::Box, double>>> &frontierRegions, int max_regions) {
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

    //Truncate the regions to the maximum amount of regions, by removing the smallest regions
    if (frontierRegions.size() > max_regions) {
        std::sort(frontierRegions.begin(), frontierRegions.end(), [](const auto &a, const auto &b) {
            return a.size() > b.size();
        });
        frontierRegions.resize(max_regions);
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
    std::vector<std::pair<quadtree::Box, double>> frontiers = agent->quadtree->queryFrontierBoxes(agent->position, agent->config.FRONTIER_SEARCH_RADIUS,
                                                                                                  agent->elapsed_ticks /
                                                                                                  agent->ticks_per_second, agent->config.MAX_FRONTIER_CELLS);
    agent->current_frontiers = frontiers;

    // Initialize an empty vector of vectors to store frontier regions
    std::vector<std::vector<std::pair<quadtree::Box, double>>> frontierRegions = {};

    mergeAdjacentFrontiers(frontiers, frontierRegions, agent->config.MAX_FRONTIER_REGIONS);


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
    if (sqrt(pow(agent->currentBestFrontier.x - agent->position.x, 2) + pow(agent->currentBestFrontier.y - agent->position.y, 2)) > agent->config.FRONTIER_DIST_UNTIL_REACHED) {
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
        double pheromoneCurve = 0;
        for (auto [box, pheromone]: region) {
            double cellsInBox = box.getSize() / agent->quadtree->getResolution();
            assert(cellsInBox == 1);
            sumX += box.getCenter().x *
                    cellsInBox; //Take the box size into account (parent nodes will contain the info about all its children)
            sumY += box.getCenter().y * cellsInBox;
            totalNumberOfCellsInRegion += cellsInBox;
            averagePheromoneCertainty += std::abs(pheromone - 0.5);
            pheromoneCurve += std::sqrt(std::abs(pheromone - 0.5));
        }
        double frontierRegionX = sumX / totalNumberOfCellsInRegion;
        double frontierRegionY = sumY / totalNumberOfCellsInRegion;
        averagePheromoneCertainty /= totalNumberOfCellsInRegion;


        argos::CVector2 vectorToFrontier = argos::CVector2(frontierRegionX - agent->position.x, frontierRegionY - agent->position.y).Rotate(-agent->heading);
        std::vector<std::pair<Coordinate, Coordinate>> route_to_frontier = {{agent->position, Coordinate{frontierRegionX, frontierRegionY}}};

        //Calculate the distance between the agent and the frontier region

        double distance = sqrt(pow(frontierRegionX - agent->position.x, 2) + pow(frontierRegionY - agent->position.y, 2));
        std::vector<argos::CVector2> relative_route = {vectorToFrontier.Rotate(-agent->heading)};
        //Relative vector to heading


        //Calculate the cost of the frontier region
        double fitness = -agent->FRONTIER_DISTANCE_WEIGHT * distance + agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
                         agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;




        //If the cost is lower than the best cost, update the best cost and best frontier region
        if (fitness > highestFrontierFitness) {
            highestFrontierFitness = fitness;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
            agent->bestFrontierRegionBoxes = region;
        }
    }


    agent->currentBestFrontier = bestFrontierRegionCenter;




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
                                                                                   agent->config.OBJECT_AVOIDANCE_RADIUS * 2.0,
                                                                                   agent->elapsed_ticks /
                                                                                   agent->ticks_per_second);

    double angleInterval = argos::CDegrees(360 / agent->config.STEPS_360_DEGREES).GetValue();

    //Create set of free angles ordered to be used for wall following
//    std::set<argos::CDegrees, CustomComparator> freeAngles(
//            CustomComparator(agent->wallFollower.wallFollowingDirection, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue()));

    std::set<argos::CDegrees> freeAngles;

//Add free angles from -180 to 180 degrees
    for (int a = 0; a < agent->config.STEPS_360_DEGREES; a++) {
        auto angle = argos::CDegrees(a * 360 / agent->config.STEPS_360_DEGREES - 180);
        freeAngles.insert(angle);
    }


    Coordinate min_real = Coordinate{agent->position.x - agent->config.OBJECT_AVOIDANCE_RADIUS, agent->position.y - agent->config.OBJECT_AVOIDANCE_RADIUS};
    Coordinate max_real = Coordinate{agent->position.x + agent->config.OBJECT_AVOIDANCE_RADIUS, agent->position.y + agent->config.OBJECT_AVOIDANCE_RADIUS};
//    int max_x_real = this->position.x + OBJECT_AVOIDANCE_RADIUS;
//    int min_y_real = this->position.y - OBJECT_AVOIDANCE_RADIUS;
//    int max_y_real = this->position.y + OBJECT_AVOIDANCE_RADIUS;

    auto[min_x_index, min_y_index] = agent->obstacleMatrix->getIndexFromRealCoordinate(min_real);
    auto[max_x_index, max_y_index] = agent->obstacleMatrix->getIndexFromRealCoordinate(max_real);

    //For each occupied box, find the angles that are blocked relative to the agent
    //We only have to look in the area of the object avoidance radius
    for (int x =std::max(0, min_x_index); x< std::min(agent->obstacleMatrix->getWidth(), max_x_index); x++){
        for (int y = std::max(0, min_y_index); y<std::min(agent->obstacleMatrix->getHeight(), max_y_index); y++) {

            Coordinate cellCenter = agent->obstacleMatrix->getRealCoordinateFromIndex(x, y);
            //If this cell is not within the object avoidance radius, skip it
            if (std::sqrt(std::pow(cellCenter.x - agent->position.x, 2) +
                          std::pow(cellCenter.y - agent->position.y, 2)) > agent->config.OBJECT_AVOIDANCE_RADIUS) {
                continue;
            }

            auto cellValue = agent->obstacleMatrix->getByIndex(x, y, elapsed_ticks / ticks_per_second);

            if (cellValue == 0) continue;


            argos::CVector2 OC = argos::CVector2(cellCenter.x - this->position.x,
                                                 cellCenter.y - this->position.y);
            argos::CRadians Bq = argos::ASin(
                    std::min(agent->config.AGENT_SAFETY_RADIUS + this->config.OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
            argos::CRadians Eta_q = OC.Angle();
            if (agent->config.AGENT_SAFETY_RADIUS + agent->config.OBJECT_SAFETY_RADIUS > OC.Length())
                argos::RLOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << agent->config.AGENT_SAFETY_RADIUS
                               << " + " << agent->config.OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;

            argos::CDegrees minAngle = ToDegrees((Eta_q - Bq).SignedNormalize());
            argos::CDegrees maxAngle = ToDegrees((Eta_q + Bq).SignedNormalize());

            if (maxAngle < minAngle) {
                argos::CDegrees temp = minAngle;
                minAngle = maxAngle;
                maxAngle = temp;
            }


            //Round to multiples of 360/this->config.STEPS_360_DEGREES
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
            for (int a = 0; a < agent->config.STEPS_360_DEGREES; a++) {
                auto angle = argos::CDegrees(a * 360 / agent->config.STEPS_360_DEGREES - 180);

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

    }

    //Block angles according to the distance sensors
    for (int i = 0; i < agent->distance_sensors.size(); i++) {
        argos::CRadians sensor_rotation = agent->heading - i * argos::CRadians::PI_OVER_TWO;
        if (agent->distance_sensors[i].getDistance() < agent->config.OBJECT_AVOIDANCE_RADIUS) {
            //Erase all angles 90 degrees to the left and right of the sensor
            //12.5% to the left
            argos::CDegrees minAngle = argos::CDegrees(int(ToDegrees(sensor_rotation - argos::CRadians::PI_OVER_SIX).GetValue())).SignedNormalize();
            argos::CDegrees maxAngle = argos::CDegrees(int(ToDegrees(sensor_rotation + argos::CRadians::PI_OVER_SIX).GetValue())).SignedNormalize();

            if (maxAngle.GetValue() < minAngle.GetValue()) {
                argos::CDegrees temp = minAngle;
                minAngle = maxAngle;
                maxAngle = temp;
            }

            auto diffMinSensor = NormalizedDifference(ToRadians(minAngle), sensor_rotation);
            auto diffMaxSensor = NormalizedDifference(ToRadians(maxAngle), sensor_rotation);


            if (diffMinSensor >= argos::CRadians(0) && diffMaxSensor <= argos::CRadians(0)) {
                for (int a = 0; a < 60; a++) {
                    auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
                    freeAngles.erase(argos::CDegrees(angle));
                }
            } else if (diffMinSensor <= argos::CRadians(0) && diffMaxSensor >= argos::CRadians(0)) {
                for (int a = 0; a < 60; a++) {
                    auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
                    freeAngles.erase(argos::CDegrees(angle));
                }
            } else {
                assert(0);
            }

//            if (minAngle.GetValue() < 0 && maxAngle.GetValue() >= 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else if (minAngle.GetValue() >= 0 && maxAngle.GetValue() >= 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (maxAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else if (minAngle.GetValue() < 0 && maxAngle.GetValue() < 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (maxAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            }
//            }
//            //Erase all in between from freeAngles
//            for (int a = int(minAngle.GetValue()); a < int(maxAngle.GetValue()); a++){
//                freeAngles.erase(argos::CDegrees(a));
//            }
        }
    }




    //If there are no free angles, see if there are any sensors that have no close intersection.
    if (freeAngles.empty()) {
        for (int i = 0; i < agent->distance_sensors.size(); i++) {
            argos::CRadians sensor_rotation = agent->heading - i * argos::CRadians::PI_OVER_TWO;
            if (agent->distance_sensors[i].getDistance() > agent->config.OBJECT_AVOIDANCE_RADIUS) {
                argos::CDegrees minAngle = argos::CDegrees(
                        int(ToDegrees(sensor_rotation - argos::CRadians::PI / 18.0).GetValue())).SignedNormalize();
                argos::CDegrees maxAngle = argos::CDegrees(int(ToDegrees(
                        sensor_rotation + argos::CRadians::PI_OVER_SIX / 18.0).GetValue())).SignedNormalize();

                if (maxAngle.GetValue() < minAngle.GetValue()) {
                    argos::CDegrees temp = minAngle;
                    minAngle = maxAngle;
                    maxAngle = temp;
                }

                auto diffMinSensor = NormalizedDifference(ToRadians(minAngle), sensor_rotation);
                auto diffMaxSensor = NormalizedDifference(ToRadians(maxAngle), sensor_rotation);


                if (diffMinSensor >= argos::CRadians(0) && diffMaxSensor <= argos::CRadians(0)) {
                    for (int a = 0; a < 10; a++) {
                        auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
                        freeAngles.insert(argos::CDegrees(angle));
                    }
                } else if (diffMinSensor <= argos::CRadians(0) && diffMaxSensor >= argos::CRadians(0)) {
                    for (int a = 0; a < 10; a++) {
                        auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
                        freeAngles.insert(argos::CDegrees(angle));
                    }
                } else {
                    assert(0);
                }
            }
        }
    }
    agent->freeAnglesVisualization.clear();
    auto closestFreeAngle = *freeAngles.begin();
    CustomComparator customComparator(0, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue());
    for (auto freeAngle: freeAngles) {
        agent->freeAnglesVisualization.insert(freeAngle);
        if (customComparator(freeAngle, closestFreeAngle)) {
            closestFreeAngle = freeAngle;
        }
    }
    //If we still have no free angles, return false
    if (freeAngles.empty()) return false;


    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);


//    if (frontier_vector_zero) return true; //If the frontier vector is zero, we must follow the force vector, so can't do wall/path following

    return true;

}

/**
 * Calculate the vector to avoid objects:
 * Find the closest relative angle that is free of objects within a certain range.
 * According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * @return The vector towards the free direction
 */
bool ForceVectorCalculator::calculateObjectAvoidanceAngle(argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {

    //Get occupied boxes within range
//    std::vector<quadtree::Box> occupiedBoxes = this->quadtree->queryOccupiedBoxes(this->position,
//                                                                                  OBJECT_AVOIDANCE_RADIUS * 2,
//                                                                                  this->elapsed_ticks /
//                                                                                  this->ticks_per_second);



    std::set<argos::CDegrees> freeAngles = {};
    double angleInterval = argos::CDegrees(360 / this->config.STEPS_360_DEGREES).GetValue();

//Add free angles from -180 to 170 degrees
    for (int a = 0; a < this->config.STEPS_360_DEGREES; a++) {
        auto angle = argos::CDegrees(a * 360 / this->config.STEPS_360_DEGREES - 180);
        freeAngles.insert(angle);
    }

    Coordinate min_real = Coordinate{this->position.x - this->config.OBJECT_AVOIDANCE_RADIUS, this->position.y - this->config.OBJECT_AVOIDANCE_RADIUS};
    Coordinate max_real = Coordinate{this->position.x + this->config.OBJECT_AVOIDANCE_RADIUS, this->position.y + this->config.OBJECT_AVOIDANCE_RADIUS};
//    int max_x_real = this->position.x + OBJECT_AVOIDANCE_RADIUS;
//    int min_y_real = this->position.y - OBJECT_AVOIDANCE_RADIUS;
//    int max_y_real = this->position.y + OBJECT_AVOIDANCE_RADIUS;

    auto[min_x_index, min_y_index] = this->obstacleMatrix->getIndexFromRealCoordinate(min_real);
    auto[max_x_index, max_y_index] = this->obstacleMatrix->getIndexFromRealCoordinate(max_real);

    //For each occupied box, find the angles that are blocked relative to the agent
    //We only have to look in the area of the object avoidance radius
    for (int x =std::max(0, min_x_index); x< std::min(this->obstacleMatrix->getWidth(), max_x_index); x++){
        for (int y = std::max(0, min_y_index); y<std::min(this->obstacleMatrix->getHeight(), max_y_index); y++) {

            Coordinate cellCenter = this->obstacleMatrix->getRealCoordinateFromIndex(x, y);
            //If this cell is not within the object avoidance radius, skip it
            if (std::sqrt(std::pow(cellCenter.x - this->position.x, 2) +
                          std::pow(cellCenter.y - this->position.y, 2)) > this->config.OBJECT_AVOIDANCE_RADIUS) {
                continue;
            }

            auto cellValue = this->obstacleMatrix->getByIndex(x, y, elapsed_ticks / ticks_per_second);

            if (cellValue == 0) continue;


            argos::CVector2 OC = argos::CVector2(cellCenter.x - this->position.x,
                                                 cellCenter.y - this->position.y);
            argos::CRadians Bq = argos::ASin(
                    std::min(this->config.AGENT_SAFETY_RADIUS + this->config.OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
            argos::CRadians Eta_q = OC.Angle();
            if (this->config.AGENT_SAFETY_RADIUS + this->config.OBJECT_SAFETY_RADIUS > OC.Length())
                argos::RLOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << this->config.AGENT_SAFETY_RADIUS
                               << " + " << this->config.OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;

            argos::CDegrees minAngle = ToDegrees((Eta_q - Bq).SignedNormalize());
            argos::CDegrees maxAngle = ToDegrees((Eta_q + Bq).SignedNormalize());

            if (maxAngle < minAngle) {
                argos::CDegrees temp = minAngle;
                minAngle = maxAngle;
                maxAngle = temp;
            }


            //Round to multiples of 360/this->config.STEPS_360_DEGREES
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
            for (int a = 0; a < this->config.STEPS_360_DEGREES; a++) {
                auto angle = argos::CDegrees(a * 360 / this->config.STEPS_360_DEGREES - 180);

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

    }

    this->freeAnglesVisualization.clear();
    for (auto freeAngle: freeAngles) {
        this->freeAnglesVisualization.insert(freeAngle);
    }

    if (freeAngles.empty()) return false;


    //Get free angle closest to heading
    auto closestFreeAngle = *freeAngles.begin();
    for (auto freeAngle: freeAngles) {
        if (std::abs((freeAngle - ToDegrees(targetAngle)).GetValue()) <
            abs((closestFreeAngle - ToDegrees(targetAngle)).GetValue())) {
            closestFreeAngle = freeAngle;
        }
    }


    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);
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
