//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include <set>
#include "agent.h"
#include <random>


Agent::Agent(std::string id) {
    this->id = id;
    this->position = {0.0, 0.0};
    this->heading = argos::CRadians(0);
    this->targetHeading = argos::CRadians(0);
    this->speed = 1;
    this->swarm_vector = argos::CVector2(0, 0);
    this->force_vector = argos::CVector2(0, 1);
    this->messages = new std::vector<std::string>(0);
    auto box = quadtree::Box(-5, 5, 10);
    this->quadtree = new quadtree::Quadtree(box);
}

int i = 0;

Agent::Agent(std::string id, Coordinate new_position) : id(id) {
    this->position = new_position;
}

void Agent::setPosition(double new_x, double new_y) {
    this->position = {new_x, new_y};
//    argos::RLOG << " is at position (" << this->position.x << ", " << this->position.y
//                << ")" << std::endl;
//    quadtree->add(Coordinate{this->position.x, this->position.y}, quadtree::Occupancy::FREE);
    i++;
//    if(i%100==0) quadtree->exportQuadtreeToFile(this->getId());
}


void Agent::setPosition(Coordinate new_position) {
    this->position = new_position;
}

void Agent::setHeading(argos::CRadians new_heading) {
    this->heading = Coordinate::ArgosHeadingToOwn(new_heading).SignedNormalize();
};

void Agent::setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *diffdrive) {
    this->diffdrive = diffdrive;
}


Coordinate Agent::getPosition() {
    return this->position;
}

std::string Agent::getId() const {
    return this->id;
}

std::string Agent::GetId() const {
    return this->id;
}

void Agent::setId(std::string new_id) {
    this->id = new_id;
}

void Agent::setSpeed(double new_speed) {
    this->speed = new_speed;
}

double Agent::getSpeed() const {
    return this->speed;
}


void Agent::print() {
    std::cout << "Agent " << this->id << " is at position (" << this->position.x << ", " << this->position.y
              << ")" << std::endl;
}

void Agent::updateMap() {
    //Update map with new information
}

void Agent::setLastRangeReadings(int index, double new_range) {
//    argos::RLOG << "Setting last range reading: " << index << ": " << new_range << std::endl;
    this->lastRangeReadings[index] = new_range;
}

void Agent::readDistanceSensor() {

}

void Agent::readInfraredSensor() {

}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2) {
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getMinSize() / 2;
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        this->quadtree->add(Coordinate{x, y}, quadtree::Occupancy::FREE, elapsed_ticks / ticks_per_second);
        x += stepX;
        y += stepY;
    }
}

/**
 * Add occupied object location to the quadtree
 * @param objectCoordinate
 */
void Agent::addObjectLocation(Coordinate objectCoordinate) {
    this->quadtree->add(objectCoordinate, quadtree::Occupancy::OCCUPIED, elapsed_ticks / ticks_per_second);
}

/**
 * Check for obstacles in front of the agent
 * If there is an obstacle within a certain range, add the free area between the agent and the obstacle to the quadtree
 * If there is no obstacle within range, add the free area between the agent and the end of the range to the quadtree
 */
void Agent::checkForObstacles() {
    for(int sensor_index=0; sensor_index < this->num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->heading - sensor_index*argos::CRadians::PI_OVER_TWO;
        if (this->lastRangeReadings[sensor_index] < PROXIMITY_RANGE) {

//        argos::RLOG << "Heading: " << this->heading << std::endl;
//        argos::RLOG << "Last range reading: " << this->lastRangeReading << std::endl;

            double opposite = argos::Sin(sensor_rotation) * this->lastRangeReadings[sensor_index];
            double adjacent = argos::Cos(sensor_rotation) * this->lastRangeReadings[sensor_index];

//        argos::RLOG << "Opposite: " << opposite << std::endl;
//        argos::RLOG << "Adjacent: " << adjacent << std::endl;

            Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
            addFreeAreaBetween(this->position, object);
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
            bool close_to_other_agent = false;
            for (const auto &agentLocation: this->agentLocations) {
                argos::CVector2 objectToAgent =
                        argos::CVector2(agentLocation.second.x, agentLocation.second.y)
                        - argos::CVector2(object.x, object.y);

                //If detected object and another agent are not close, add the object as an obstacle
                if (objectToAgent.Length() <= this->quadtree->getMinSize()) {
                    close_to_other_agent = true;
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if(!close_to_other_agent) addObjectLocation(object);


        } else {
            double opposite = argos::Sin(sensor_rotation) * PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * PROXIMITY_RANGE;

//        argos::RLOG << "Opposite: " << opposite << std::endl;
//        argos::RLOG << "Adjacent: " << adjacent << std::endl;

            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
            addFreeAreaBetween(this->position, end_of_ray);
        }
    }
}

/**
 * Calculate the vector to avoid objects:
 * Find the closest relative angle that is free of objects within a certain range.
 * According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * @return The vector towards the free direction
 */
bool Agent::calculateObjectAvoidanceAngle(argos::CRadians* relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {

    //Get occupied boxes within range
    std::vector<quadtree::Box> occupiedBoxes = this->quadtree->queryOccupiedBoxes(this->position, OBJECT_AVOIDANCE_RADIUS*2,
                                                                                  this->elapsed_ticks /
                                                                                  this->ticks_per_second);

    std::set<argos::CDegrees> blockedAngles = {};
    std::set<argos::CDegrees> freeAngles = {};
    double angleInterval = argos::CDegrees(360 / ANGLE_INTERVAL_STEPS).GetValue();

//Add free angles from -180 to 170 degrees
    for (int a = 0; a < ANGLE_INTERVAL_STEPS; a++) {
        auto angle = argos::CDegrees(a * 360 / ANGLE_INTERVAL_STEPS - 180);
        freeAngles.insert(angle);
    }

//    argos::RLOG << "Occupied boxes:" << occupiedBoxes.size() << std::endl;
    //For each occupied box, find the angles that are blocked relative to the agent
    for (auto box: occupiedBoxes) {

//        argos::RLOG << "Box: " << box.getCenter().x << " ; " << box.getCenter().y << std::endl;

        argos::CVector2 OC = argos::CVector2(box.getCenter().x - this->position.x,
                                             box.getCenter().y - this->position.y);
        argos::CRadians Bq = argos::ASin(
                std::min(AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
        argos::CRadians Eta_q = OC.Angle();
        if(AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS > OC.Length())
        argos::RLOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << AGENT_SAFETY_RADIUS << " + " << OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;


//        argos::RLOG << "OC: " << OC.Length() << std::endl;
//
//        argos::RLOG << "Bq calc: (" << AGENT_SAFETY_RADIUS << '+' << OBJECT_SAFETY_RADIUS <<")/" << OC.Length() << " = " << Bq << std::endl;
//        argos::RLOG << "Heading: " << ToDegrees(this->heading) << std::endl;
//        argos::RLOG << "OC angle: " << ToDegrees(OC.Angle()) << std::endl;
//        argos::RLOG << "Eta_q: " << ToDegrees(Eta_q) << std::endl;

//        //Get the relative angle of the corners of the box
//        argos::CVector2 vectorToObjectTopLeft =
//                argos::CVector2(box.left, box.top) - argos::CVector2(this->position.x, this->position.y);
//        argos::CVector2 vectorToObjectTopRight =
//                argos::CVector2(box.getRight(), box.top) - argos::CVector2(this->position.x, this->position.y);
//        argos::CVector2 vectorToObjectBottomLeft =
//                argos::CVector2(box.left, box.getBottom()) - argos::CVector2(this->position.x, this->position.y);
//        argos::CVector2 vectorToObjectBottomRight =
//                argos::CVector2(box.getRight(), box.getBottom()) - argos::CVector2(this->position.x, this->position.y);
//
//        argos::CDegrees angleToObjectTopLeft = ToDegrees(
//                vectorToObjectTopLeft.Angle() - this->heading).SignedNormalize();
//        argos::CDegrees angleToObjectTopRight = ToDegrees(
//                vectorToObjectTopRight.Angle() - this->heading).SignedNormalize();
//        argos::CDegrees angleToObjectBottomLeft = ToDegrees(
//                vectorToObjectBottomLeft.Angle() - this->heading).SignedNormalize();
//        argos::CDegrees angleToObjectBottomRight = ToDegrees(
//                vectorToObjectBottomRight.Angle() - this->heading).SignedNormalize();



//
//        //Find min and maximum angle so that you can find the blocked range
//        argos::CDegrees minAngle = std::min(
//                {angleToObjectTopLeft, angleToObjectTopRight, angleToObjectBottomLeft, angleToObjectBottomRight});
//        argos::CDegrees maxAngle = std::max(
//                {angleToObjectTopLeft, angleToObjectTopRight, angleToObjectBottomLeft, angleToObjectBottomRight});


        argos::CDegrees minAngle = ToDegrees((Eta_q - Bq).SignedNormalize());
        argos::CDegrees maxAngle = ToDegrees((Eta_q + Bq).SignedNormalize());

        if (maxAngle < minAngle) {
            argos::CDegrees temp = minAngle;
            minAngle = maxAngle;
            maxAngle = temp;
        }

//        argos::RLOG << "Min angle: " << minAngle << std::endl;
//        argos::RLOG << "Max angle: " << maxAngle << std::endl;


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
//        argos::RLOG << "Min angle rounded: " << roundedMinAngle << std::endl;
//        argos::RLOG << "Max angle rounded: " << roundedMaxAngle << std::endl;

        //Block all angles within range
        for (int a = 0; a < ANGLE_INTERVAL_STEPS; a++) {
            auto angle = argos::CDegrees(a * 360 / ANGLE_INTERVAL_STEPS - 180);
//            argos::RLOG << "Angle: " << angle << std::endl;
//            argos::RLOG << "Diff: " << (roundedMaxAngle - roundedMinAngle) << std::endl;
//            argos::RLOG << "Normalized diff minangle etaq: " << NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) << std::endl;
//            argos::RLOG << "Normalized diff maxangle etaq: " << NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) << std::endl;
//            if (minAngle <= ToDegrees(Eta_q) && ToDegrees(Eta_q) <= maxAngle) {
            if(NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) <= argos::CRadians(0) && NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) >= argos::CRadians(0)) {
                if (angle >= roundedMinAngle && angle <= roundedMaxAngle) {
//                    blockedAngles.insert(angle);
                    freeAngles.erase(angle);
//                    argos::RLOG << "Blocked angle: " << angle << std::endl;
                }
//                else {
//                    freeAngles.insert(angle);
//                    argos::RLOG << "Free angle: " << angle << std::endl;
//                }
            } else if(NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) >= argos::CRadians(0) && NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) <= argos::CRadians(0)) {

                if (angle <= roundedMinAngle || angle >= roundedMaxAngle) {
//                    blockedAngles.insert(angle);
                    freeAngles.erase(angle);
//                    argos::RLOG << "Blocked angle: " << angle << std::endl;
                }
//                else {
////                    freeAngles.insert(angle);
//                    argos::RLOG << "Free angle: " << angle << std::endl;
//                }

            } else {
                argos::LOGERR << "Error: Eta_q not within range" << std::endl;
            }
        }

    }


//    //Find free angles
//    for (int a = 0; a < ANGLE_INTERVAL_STEPS; a++) {
//        auto angle = argos::CDegrees(a * 360 / ANGLE_INTERVAL_STEPS);
//        if (blockedAngles.find(angle) == blockedAngles.end()) {
////            freeAngles.insert(angle.SignedNormalize());
//            argos::RLOG << "Free angle: " << angle << std::endl;
//        }
//    }

//    argos::RLOG <<"Heading: " << ToDegrees(this->heading) << std::endl;

    if(freeAngles.empty()) return false;

//    argos::RLOG << "Target angle: " <<ToDegrees(targetAngle) << std::endl;

    //Get free angle closest to heading
    auto closestFreeAngle = * freeAngles.begin();
    for (auto freeAngle: freeAngles) {
//        argos::RLOG << "Free angle: " << freeAngle << std::endl;
        if (std::abs((freeAngle - ToDegrees(targetAngle)).GetValue()) <
            abs((closestFreeAngle - ToDegrees(targetAngle)).GetValue())) {
            closestFreeAngle = freeAngle;
        }
    }

//    argos::RLOG << "Closest free angle: " << closestFreeAngle << std::endl;
//    argos::RLOG << "Target angle: " << ToDegrees(targetAngle) << std::endl;

    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);
    return true;
    //Create a vector towards the closestFreeAngle
//    double opposite = argos::Sin(this->heading + closestFreeAngleRadians) * 1.0;
//    double adjacent = argos::Cos(this->heading + closestFreeAngleRadians) * 1.0;
//    argos::CVector2 vectorToFreeAngle = argos::CVector2(adjacent, opposite);
//    return vectorToFreeAngle;

}

/** Calculate the vector to avoid the virtual walls
 * If the agent is close to the border, create a vector pointing away from the border
 * Implemented according to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * However, the vector directions are flipped compared to the paper, as the paper uses a different coordinate system
 * @return a vector pointing away from the border
 */
argos::CVector2 Agent::getVirtualWallAvoidanceVector(){
    //If the agent is close to the border, create a vector pointing away from the border
    argos::CVector2 virtualWallAvoidanceVector = {0, 0};

    if(this->position.x < this->left_right_borders.x){
        virtualWallAvoidanceVector.SetX(1);
    } else if (this->left_right_borders.x <= this->position.x && this->position.x <= this->left_right_borders.y){
        virtualWallAvoidanceVector.SetX(0);
    } else if(this->position.x > this->left_right_borders.y){
        virtualWallAvoidanceVector.SetX(-1);
    }

    if(this->position.y < this->upper_lower_borders.y){
        virtualWallAvoidanceVector.SetY(1);
    } else if (this->upper_lower_borders.y <= this->position.y && this->position.y <= this->upper_lower_borders.x){
        virtualWallAvoidanceVector.SetY(0);
    } else if(this->position.y > this->upper_lower_borders.x){
        virtualWallAvoidanceVector.SetY(-1);
    }


    return virtualWallAvoidanceVector;

}

bool Agent::getAverageNeighborLocation(Coordinate *averageNeighborLocation) {
    int nAgentsWithinRange = 0;
    for (const auto &agentLocation: this->agentLocations) {
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(agentLocation.second.x, agentLocation.second.y)
                - argos::CVector2(this->position.x, this->position.y);

        if (vectorToOtherAgent.Length() < AGENT_AVOIDANCE_RANGE) { //TODO: make different for cohesion and separation
            averageNeighborLocation->x += agentLocation.second.x;
            averageNeighborLocation->y += agentLocation.second.y;
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

argos::CVector2 Agent::calculateAgentCohesionVector() {
    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(&averageNeighborLocation);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between this agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(this->position.x, this->position.y);

    return vectorToOtherAgents;
}


/**
 * Calculate the vector to avoid other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the opposite direction of the average location of these agents.
 * @return a vector pointing away from the average location other agents
 */
argos::CVector2 Agent::calculateAgentAvoidanceVector(argos::CVector2 agentCohesionVector) {

    return agentCohesionVector * -1;
}

/**
 * Calculate the vector to align with other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the direction of the average vector of these agents, considering speed.
 * @return
 */
argos::CVector2 Agent::calculateAgentAlignmentVector() {
    argos::CVector2 alignmentVector = {0, 0};
    int nAgentsWithinRange = 0;

//    argos::RLOG << "Agent velocities size: " << this->agentVelocities.size() << std::endl;

    //Get the velocities of the agents within range
    for (const auto &agentVelocity: agentVelocities) {
        std::string agentID = agentVelocity.first;
        Coordinate otherAgentLocation = agentLocations[agentID];
        argos::CVector2 agentVector = agentVelocity.second.first;
        double agentSpeed = agentVelocity.second.second;
        argos::CVector2 vectorToOtherAgent = argos::CVector2(otherAgentLocation.x, otherAgentLocation.y)
                                             - argos::CVector2(this->position.x, this->position.y);
        if (vectorToOtherAgent.Length() < AGENT_ALIGNMENT_RANGE) {
//            argos::RLOG << "Agent within range" << std::endl;
//            argos::RLOG << "Agent vector: " << agentVector << std::endl;
//            argos::RLOG << "Agent speed: " << agentSpeed << std::endl;
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

argos::CVector2 Agent::calculateUnexploredFrontierVector() {
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
    std::vector<quadtree::Box> frontiers = this->quadtree->queryFrontierBoxes(this->position, FRONTIER_SEARCH_DIAMETER,
                                                                              this->elapsed_ticks /
                                                                              this->ticks_per_second);

    // Initialize an empty vector of vectors to store frontier regions
    std::vector<std::vector<quadtree::Box>> frontierRegions = {};

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
                // the diagonal of a box with minimum size (ensuring adjacency)
                if (sqrt(pow(boxCenter.x - frontierCenter.x, 2) + pow(boxCenter.y - frontierCenter.y, 2)) <=
                    sqrt(2 * pow(this->quadtree->getMinSize(), 2))) {
                    region.push_back(frontier); // Add the frontier to the current region
                    added = true; // Mark the frontier as added
                    break; // Exit the loop since the frontier has been added to a region
                }
            }
//            if (added) break; // If the frontier has been added to a region, exit the loop
        }

        // If the frontier was not added to any existing region, create a new region with it
        if (!added) {
            frontierRegions.push_back({frontier});
        }
    }

    //Now we have all frontier cells merged into frontier regions
    //Find F* by using the formula above
    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
    //Ψ_S = FRONTIER_SIZE_WEIGHT

    //Initialize variables to store the best frontier region and its score
    std::vector<quadtree::Box> bestFrontierRegion = {};
    Coordinate bestFrontierRegionCenter = {0, 0};
    double bestFrontierScore = std::numeric_limits<double>::max();

    //Iterate over all frontier regions to find the best one
    for (auto region: frontierRegions) {
        //Calculate the average position of the frontier region
        double sumX = 0;
        double sumY = 0;
        for (auto box: region) {
            sumX += box.getCenter().x;
            sumY += box.getCenter().y;
        }
        double frontierRegionSize = region.size();
        double frontierRegionX = sumX / frontierRegionSize;
        double frontierRegionY = sumY / frontierRegionSize;

        //Calculate the distance between the agent and the frontier region
        double distance = sqrt(pow(frontierRegionX - this->position.x, 2) + pow(frontierRegionY - this->position.y, 2));

        //Calculate the score of the frontier region
        double score = FRONTIER_DISTANCE_WEIGHT * distance - FRONTIER_SIZE_WEIGHT * frontierRegionSize;

        //If the score is lower than the best score, update the best score and best frontier region
        if (score < bestFrontierScore) {
            bestFrontierScore = score;
            bestFrontierRegion = region;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
        }
    }

    this->currentBestFrontier = bestFrontierRegionCenter;

//    argos::RLOG << "Best frontier region center: " << bestFrontierRegionCenter.x << ", " << bestFrontierRegionCenter.y
//                << std::endl;

    //Calculate the vector to the best frontier region
    argos::CVector2 vectorToBestFrontier = argos::CVector2(bestFrontierRegionCenter.x, bestFrontierRegionCenter.y)
                                           - argos::CVector2(this->position.x, this->position.y);

    return vectorToBestFrontier;
//    return {0, 0};
}

void Agent::calculateNextPosition() {
    //Inspired by boids algorithm:
    //Vector determining heading
    //Vector is composed of:
    //1. Attraction to unexplored frontier
    //2. Repulsion from other agents (done)
    //3. Attraction to found target
    //4. Repulsion from objects/walls (done)


    argos::CVector2 virtualWallAvoidanceVector = getVirtualWallAvoidanceVector();
    argos::CVector2 agentCohesionVector = calculateAgentCohesionVector();
    argos::CVector2 agentAvoidanceVector = calculateAgentAvoidanceVector(agentCohesionVector);
    argos::CVector2 agentAlignmentVector = calculateAgentAlignmentVector();

    argos::CVector2 unexploredFrontierVector = calculateUnexploredFrontierVector();

//    argos::RLOG << "Virtual wall avoidance vector angle: " << virtualWallAvoidanceVector.Angle() << std::endl;
//    argos::RLOG << "Agent avoidance vector angle: " << agentAvoidanceVector.Angle() << std::endl;
//    argos::RLOG << "Agent cohesion vector angle: " << agentCohesionVector.Angle() << std::endl;
//    argos::RLOG << "Unexplored frontier vector angle: " << unexploredFrontierVector.Angle() << std::endl;

    //If there are agents to avoid, do not explore
    if (agentAvoidanceVector.Length() != 0) unexploredFrontierVector = {0, 0};


    //Normalize vectors if they are not zero
//    if (objectAvoidanceVector.Length() != 0) objectAvoidanceVector.Normalize();
    if (virtualWallAvoidanceVector.Length() != 0) virtualWallAvoidanceVector.Normalize();
    if (agentCohesionVector.Length() != 0) agentCohesionVector.Normalize();
    if (agentAvoidanceVector.Length() != 0) agentAvoidanceVector.Normalize();
    if (agentAlignmentVector.Length() != 0) agentAlignmentVector.Normalize();
    if (unexploredFrontierVector.Length() != 0) unexploredFrontierVector.Normalize();

    virtualWallAvoidanceVector = VIRTUAL_WALL_AVOIDANCE_WEIGHT * virtualWallAvoidanceVector;
    agentCohesionVector = AGENT_COHESION_WEIGHT * agentCohesionVector; //Normalize first
    agentAvoidanceVector = AGENT_AVOIDANCE_WEIGHT * agentAvoidanceVector;
    agentAlignmentVector = AGENT_ALIGNMENT_WEIGHT * agentAlignmentVector;
    unexploredFrontierVector = UNEXPLORED_FRONTIER_WEIGHT * unexploredFrontierVector;

    //According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    argos::CVector2 total_vector = this->swarm_vector +
                                   //                                   + OBJECT_AVOIDANCE_WEIGHT * objectAvoidanceVector +
                                   virtualWallAvoidanceVector +
                                   agentCohesionVector +
                                   agentAvoidanceVector +
                                   agentAlignmentVector +
                                   unexploredFrontierVector;
    if (total_vector.Length() != 0) total_vector.Normalize();

    this->swarm_vector = total_vector;
//    argos::RLOG << "Swarm vector: " << this->swarm_vector << std::endl;
    argos::CRadians objectAvoidanceAngle;
    bool isThereAFreeAngle = calculateObjectAvoidanceAngle(&objectAvoidanceAngle, this->swarm_vector.Angle());
    //If there is not a free angle to move to, do not move
    if(!isThereAFreeAngle){
        this->force_vector = {0,0};
//        argos::RLOG << "No free angle to move to" << std::endl;
    }
    else {
//        argos::RLOG <<"Object avoidance angle:" << objectAvoidanceAngle << std::endl;
//        argos::RLOG << "Swarm vector angle" << this->swarm_vector.Angle() << std::endl;
//
//

//
//        argos::RLOG << "Object avoidance angle: " << objectAvoidanceAngle << std::endl;
//        argos::RLOG << "Object avoidance angle 0-2PI: " << objectAvoidanceAngle.UnsignedNormalize() << std::endl;
//
//        argos::RLOG << "Force vector calculation: {(" << this->swarm_vector.GetX()
////                << " + " << unexploredFrontierVector.GetX()
//                    << ") * cos(" << objectAvoidanceAngle << ") - ("
//                    << this->swarm_vector.GetX()
//                    << " + " << unexploredFrontierVector.GetX()
//                    << ") * sin(" << objectAvoidanceAngle << "), "
//                    << "(" << this->swarm_vector.GetY()
////                    << " + " << unexploredFrontierVector.GetY()
//                    << ") * sin(" << objectAvoidanceAngle << ") + ("
//                    << this->swarm_vector.GetY()
////                    << " + "
//                    << ") * cos(" << objectAvoidanceAngle << ")}" << std::endl;


// According to the paper, the formula should be:
//        this->force_vector = {(this->swarm_vector.GetX()) *
//                              argos::Cos(objectAvoidanceAngle) -
//                              (this->swarm_vector.GetX()) *
//                              argos::Sin(objectAvoidanceAngle),
//                              (this->swarm_vector.GetY()) *
//                              argos::Sin(objectAvoidanceAngle) +
//                              (this->swarm_vector.GetY()) *
//                              argos::Cos(objectAvoidanceAngle)};

// However, according to theory (https://en.wikipedia.org/wiki/Rotation_matrix) (https://www.purplemath.com/modules/idents.htm), the formula should be:
// This also seems to give better performance.
        this->force_vector = {(this->swarm_vector.GetX()) *
                              argos::Cos(objectAvoidanceAngle) -
                              (this->swarm_vector.GetY()) *
                              argos::Sin(objectAvoidanceAngle),
                              (this->swarm_vector.GetX()) *
                              argos::Sin(objectAvoidanceAngle) +
                              (this->swarm_vector.GetY()) *
                              argos::Cos(objectAvoidanceAngle)};
//        argos::RLOG << "Force vector: " << this->force_vector << std::endl;
//        argos::RLOG << "Force vector angle" << this->force_vector.Angle() << std::endl;
//
//        argos::RLOG << "Heading: " << this->heading << std::endl;

        argos::CRadians angle = this->force_vector.Angle();
        this->targetHeading = angle;
    }
}

void Agent::doStep() {
    broadcastMessage("C:" + this->position.toString());
    std::vector<std::string> quadTreeToStrings = {};
    this->quadtree->toStringVector(&quadTreeToStrings);
//    argos::RLOG << "quadTreeStringSize: " << quadTreeToStrings.size() << std::endl;
    for (const std::string &str: quadTreeToStrings) {
        broadcastMessage("M:" + str);
    }
    broadcastMessage(
            "V:" + std::to_string(this->force_vector.GetX()) + ";" + std::to_string(this->force_vector.GetY()) +
            ":" + std::to_string(this->speed));

    checkMessages();

    checkForObstacles();

    calculateNextPosition();

    //If there is no force vector, do not move
    if(this->force_vector == argos::CVector2{0,0}) this->diffdrive->SetLinearVelocity(0,0);
    else {
//        argos::RLOG << "Heading: " << ToDegrees(this->heading) << std::endl;
//        argos::RLOG << "Target heading: " << ToDegrees(this->targetHeading) << std::endl;

        argos::CRadians diff = (this->heading - this->targetHeading).SignedNormalize();

        argos::CDegrees diffDeg = ToDegrees(diff);

//        argos::RLOG << "Diff: " << diffDeg << std::endl;

        if (diffDeg > argos::CDegrees(-TURN_THRESHOLD_DEGREES) && diffDeg < argos::CDegrees(TURN_THRESHOLD_DEGREES)) {
            //Go straight
            this->diffdrive->SetLinearVelocity(this->speed, this->speed);
//        argos::RLOG << "Going straight" << std::endl;
        } else if (diffDeg > argos::CDegrees(0)) {
            //turn right
            this->diffdrive->SetLinearVelocity(this->speed * TURNING_SPEED_RATIO, 0);
//        argos::RLOG << "Turning right" << std::endl;

        } else {
            //turn left
            this->diffdrive->SetLinearVelocity(0, this->speed * TURNING_SPEED_RATIO);
//        argos::RLOG << "Turning left" << std::endl;

        }
    }

//    argos::RLOG << std::endl;

    elapsed_ticks++;
}


/**
 * Broadcast a message to all agents
 * @param message
 */
void Agent::broadcastMessage(std::string message) {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    argos::UInt8 *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    this->wifi.broadcast_message(cMessage);
}

/**
 * Check for messages from other agents
 * If there are messages, parse them
 */
void Agent::checkMessages() {
    //Read messages from other agents
    this->wifi.receive_messages(this->messages);
    if (!this->messages->empty()) parseMessages();

}

/**
 * Get the id from a message
 * @param message
 * @return
 */
std::string getIdFromMessage(std::string message) {
    return message.substr(1, message.find(']') - 1);

}

/**
 * Get a coordinate from a string
 * @param str
 * @return
 */
Coordinate coordinateFromString(std::string str) {
    std::string delimiter = ";";
    size_t pos = 0;
    std::string token;
    pos = str.find(delimiter);
    token = str.substr(0, pos);
    Coordinate newCoordinate;
    newCoordinate.x = std::stod(token);
    str.erase(0, pos + delimiter.length());
    newCoordinate.y = std::stod(str);
    return newCoordinate;
}

/**
 * Get a QuadNode from a string
 * @param str
 * @return
 */
quadtree::QuadNode quadNodeFromString(std::string str) {
    std::string visitedDelimiter = "@";
    std::string occDelimiter = ":";
    size_t visitedPos = 0;
    size_t occPos = 0;
    std::string coordinate;
    std::string occ;
    std::string visited;
    occPos = str.find(occDelimiter);
    coordinate = str.substr(0, occPos);
    quadtree::QuadNode newQuadNode;
//    argos::LOG << "Coordinate: " << coordinate << std::endl;
    newQuadNode.coordinate = coordinateFromString(coordinate);
    str.erase(0, occPos + occDelimiter.length());
    //Now we have the occupancy and ticks

    visitedPos = str.find(visitedDelimiter);
    occ = str.substr(0, visitedPos);
//    argos::LOG << "Occupancy: " << occ << std::endl;
    newQuadNode.occupancy = static_cast<quadtree::Occupancy>(std::stoi(occ));
    str.erase(0, visitedPos + visitedDelimiter.length());
    visited = str;
    newQuadNode.visitedAtS = std::stod(visited);
//    argos::LOG << "Ticks: " << ticks << std::endl;
    return newQuadNode;
}

argos::CVector2 vector2FromString(std::string str) {
    std::string delimiter = ";";
    size_t pos = 0;
    std::string token;
    pos = str.find(delimiter);
    token = str.substr(0, pos);
    argos::CVector2 newVector;
    newVector.SetX(std::stod(token));
    str.erase(0, pos + delimiter.length());
    newVector.SetY(std::stod(str));
    return newVector;
}


/**
 * Parse messages from other agents
 */
void Agent::parseMessages() {
    for (std::string message: *this->messages) {
//        argos::RLOG << "Received message: " << message << std::endl;
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent[0] == 'C') {
            Coordinate receivedPosition = coordinateFromString(messageContent.substr(2));
            this->agentLocations[senderId] = receivedPosition;
        } else if (messageContent[0] == 'M') {
            this->quadtree->add(quadNodeFromString(messageContent.substr(2)));
        } else if (messageContent[0] == 'V') {
            std::string vectorString = messageContent.substr(2);
            std::string delimiter = ":";
            size_t speedPos = 0;
            std::string vector;
            speedPos = vectorString.find(delimiter);
            vector = vectorString.substr(0, speedPos);
            argos::CVector2 newVector = vector2FromString(vector);
            vectorString.erase(0, speedPos + delimiter.length());
            double newSpeed = std::stod(vectorString);
            agentVelocities[senderId] = {newVector, newSpeed};
        }
    }

}

Radio Agent::getWifi() const {
    return this->wifi;
}

void Agent::setWifi(Radio wifi) {
    this->wifi = wifi;

}

std::vector<std::string> Agent::getMessages() {
    return *this->messages;
}
