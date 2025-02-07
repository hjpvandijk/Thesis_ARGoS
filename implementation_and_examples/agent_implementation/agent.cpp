//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include <set>
#include "agent.h"
#include "utils/coordinate.h"
#include "utils/Algorithms.h"
#include <random>
#include <utility>
#include <unordered_map>
#include <queue>
#include <array>
#include <yaml-cpp/yaml.h>

Agent::Agent(std::string id) {
    loadConfig();
    this->id = std::move(id);
    this->position = {0.0, 0.0};
    this->heading = argos::CRadians(0);
    this->targetHeading = argos::CRadians(0);
    this->speed = 1;
    this->swarm_vector = argos::CVector2(0, 0);
    this->force_vector = argos::CVector2(0, 1);
    this->messages = std::vector<std::string>(0);
//    auto box = quadtree::Box(-5, 5, 10);
//    this->quadtree = std::make_unique<quadtree::Quadtree>(box);
    #ifdef USING_CONFIDENCE_TREE
    #else
    this->coverageMatrix = std::make_unique<PheromoneMatrix>(11, 11, this->config.COVERAGE_MATRIX_RESOLUTION, this->config.COVERAGE_MATRIX_EVAPORATION_TIME_S);
    this->obstacleMatrix = std::make_unique<PheromoneMatrix>(11, 11, this->config.OBSTACLE_MATRIX_RESOLUTION, this->config.OBSTACLE_MATRIX_EVAPORATION_TIME_S);
    #endif
    //https://e-puck.gctronic.com/index.php?option=com_content&view=article&id=7&Itemid=9
    //Motor stall values based on the tt dc gearbox motor (https://www.sgbotic.com/index.php?dispatch=products.view&product_id=2674)
    this->batteryManager = BatteryManager(this->config.ROBOT_WEIGHT, this->config.ROBOT_WHEEL_RADIUS, this->config.ROBOT_INTER_WHEEL_DISTANCE, this->config.MOTOR_STALL_TORQUE, this->config.MOTOR_NO_LOAD_RPM, this->config.MOTOR_STALL_CURRENT, this->config.MOTOR_NO_LOAD_CURRENT, this->config.BATTERY_VOLTAGE, this->config.BATTERY_CAPACITY);
    //Set the speed to the maximum achievable speed, based on the the motor specs. TODO: Put that info in differential drive instead
    auto max_achievable_speed = this->batteryManager.motionSystemBatteryManager.getMaxAchievableSpeed();
    this->differential_drive = DifferentialDrive(std::min(max_achievable_speed, this->speed), std::min(max_achievable_speed, this->speed*this->config.TURNING_SPEED_RATIO));
    this->speed = this->differential_drive.max_speed_straight;
    //Set the voltage to the voltage required for the current speed, and corresponding values, to use in calculations.
    this->batteryManager.motionSystemBatteryManager.calculateVoltageAtSpeed(this->speed);
    this->timeSynchronizer = TimeSynchronizer();
    this->sensor_reading_distance_probability = (1-this->config.P_AT_MAX_SENSOR_RANGE) / this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
}


void Agent::setPosition(double new_x, double new_y) {
    this->position = {new_x, new_y};
}


void Agent::setPosition(Coordinate new_position) {
    this->position = new_position;
}

void Agent::setHeading(argos::CRadians new_heading) {
    this->heading = Coordinate::ArgosHeadingToOwn(new_heading).SignedNormalize();
}

//void Agent::setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *newDiffdrive) {
//    this->diffdrive = newDiffdrive;
//}


Coordinate Agent::getPosition() const {
    return this->position;
}

std::string Agent::getId() const {
    return this->id;
}

std::string Agent::GetId() const {
    return this->id;
}

void Agent::setId(std::string new_id) {
    this->id = std::move(new_id);
}

void Agent::setSpeed(double new_speed) {
    this->speed = new_speed;
}

double Agent::getSpeed() const {
    return this->speed;
}


void Agent::print() const {
    std::cout << "Agent " << this->id << " is at position (" << this->position.x << ", " << this->position.y
              << ")" << std::endl;
}

void Agent::updateMap() {
    //Update map with new information
}

void Agent::setLastRangeReadings(int index, double new_range) {
    this->distance_sensors.at(index).setDistance(new_range);
}

void Agent::readDistanceSensor() {

}

void Agent::readInfraredSensor() {

}

#ifdef USING_CONFIDENCE_TREE
/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox,
                               float Psensor) {//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->coverageMatrix->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
////        this->quadtree->add(Coordinate{x, y}, quadtree::Occupancy::FREE, elapsed_ticks / ticks_per_second);
//        //If the cell is occupied, don't set area as free
//        if (this->obstacleMatrix->get(x, y, elapsed_ticks/ticks_per_second) > 0) {
//            continue;
//        }
//        this->coverageMatrix->update(x, y, elapsed_ticks/ticks_per_second);
//
//        x += stepX;
//        y += stepY;
//    }

    double dist = sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2));
    #ifdef USING_CONFIDENCE_TREE
    if (dist < this->quadtree->getResolution())
        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
    #else
    if (dist < this->coverageMatrix->getResolution())
        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
    #endif

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(this, coordinate1,
                                                                                    coordinate2);
    double step_size_avg = dist / linePoints.size();
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        if (!objectBox.contains(Coordinate{point.x, point.y})) { //Don't add a coordinate in the objectBox as free
            double p_distance_reading = this->config.P_AT_MAX_SENSOR_RANGE - double(i)*step_size_avg * sensor_reading_distance_probability + this->config.P_AT_MAX_SENSOR_RANGE;
            double p = (this->config.P_FREE - 0.5) * p_distance_reading * Psensor +
                       0.5; //Increasingly more uncertain the further away from the agent, as error can increase with position and orientation estimation inaccuracies.
//            argos::LOG << "P = " << p << "with " << i << "/" << linePoints.size() << std::endl;
            #ifdef USING_CONFIDENCE_TREE
            //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
            this->quadtree->add(Coordinate{point.x + 0.0000000001, point.y + 0.0000000001}, p,
                                elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
            #else
            if (this->obstacleMatrix->get(point.x, point.y, elapsed_ticks/ticks_per_second) > 0) {
                continue;
            }
            this->coverageMatrix->update(point.x, point.y, elapsed_ticks/ticks_per_second);
            #endif

        } else {
            break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
        }
    }
}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, float Psensor) {
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
//        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//        this->quadtree->add(Coordinate{x + 0.0000000001, y + 0.0000000001}, this->config.P_FREE * Psensor,
//                            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
//        x += stepX;
//        y += stepY;
//    }
    double dist = sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2));

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(this, coordinate1,
                                                                                    coordinate2);
    double step_size_avg = dist / linePoints.size();
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        double p_distance_reading = this->config.P_AT_MAX_SENSOR_RANGE - double(i)*step_size_avg * sensor_reading_distance_probability + this->config.P_AT_MAX_SENSOR_RANGE;
        double p = (this->config.P_FREE - 0.5) * p_distance_reading * Psensor +
                   0.5; //Increasingly more uncertain the further away from the agent, as error can increase with position and orientation estimation inaccuracies.
//        argos::LOG << "P = " << p << "with " << i << "/" << linePoints.size() << std::endl;
        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
        this->quadtree->add(Coordinate{point.x + 0.0000000001, point.y + 0.0000000001}, p,
                            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
    }
}
#else
/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2,
                               float Psensor) {//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->coverageMatrix->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
////        this->quadtree->add(Coordinate{x, y}, quadtree::Occupancy::FREE, elapsed_ticks / ticks_per_second);
//        //If the cell is occupied, don't set area as free
//        if (this->obstacleMatrix->get(x, y, elapsed_ticks/ticks_per_second) > 0) {
//            continue;
//        }
//        this->coverageMatrix->update(x, y, elapsed_ticks/ticks_per_second);
//
//        x += stepX;
//        y += stepY;
//    }

    double dist = sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2));
    #ifdef USING_CONFIDENCE_TREE
    if (dist < this->quadtree->getResolution())
        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
    #else
    if (dist < this->coverageMatrix->getResolution())
        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
    #endif

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(this, coordinate1,
                                                                                    coordinate2);
    double step_size_avg = dist / linePoints.size();
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        if (!objectBox.contains(Coordinate{point.x, point.y})) { //Don't add a coordinate in the objectBox as free
            double p_distance_reading = this->config.P_AT_MAX_SENSOR_RANGE - double(i)*step_size_avg * sensor_reading_distance_probability + this->config.P_AT_MAX_SENSOR_RANGE;
            double p = (this->config.P_FREE - 0.5) * p_distance_reading * Psensor +
                       0.5; //Increasingly more uncertain the further away from the agent, as error can increase with position and orientation estimation inaccuracies.
//            argos::LOG << "P = " << p << "with " << i << "/" << linePoints.size() << std::endl;
            #ifdef USING_CONFIDENCE_TREE
            //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
            this->quadtree->add(Coordinate{point.x + 0.0000000001, point.y + 0.0000000001}, p,
                                elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
            #else
            if (this->obstacleMatrix->get(point.x, point.y, elapsed_ticks/ticks_per_second) > 0) {
                continue;
            }
            this->coverageMatrix->update(point.x, point.y, elapsed_ticks/ticks_per_second);
            #endif

        } else {
            break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
        }
    }
}
#endif

#ifdef USING_CONFIDENCE_TREE
/**
 * Add occupied object location to the quadtree
 * @param objectCoordinate
 */
quadtree::Box Agent::addObjectLocation(Coordinate objectCoordinate, float Psensor) {
    //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
    quadtree::Box objectBox = this->quadtree->add(
            Coordinate{objectCoordinate.x + 0.0000000001, objectCoordinate.y + 0.0000000001}, this->config.P_OCCUPIED /
                                                                                              Psensor, //Divided by sensor accuracy probability (so higher resulting probability) as maybe the object is not there
            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
    return objectBox;
}
#else
/**
 * Add occupied object location to the quadtree
 * @param objectCoordinate
 */
void Agent::addObjectLocation(Coordinate objectCoordinate) {
    this->obstacleMatrix->update(objectCoordinate, elapsed_ticks/ticks_per_second);
    //Then set the same cell in the coverage matrix to -1
    this->coverageMatrix->reset(objectCoordinate);
}
#endif

/**
 * Check for obstacles in front of the agent
 * If there is an obstacle within a certain range, add the free area between the agent and the obstacle to the quadtree
 * If there is no obstacle within range, add the free area between the agent and the end of the range to the quadtree
 */
void Agent::checkForObstacles() {
//    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
//        argos::CRadians sensor_rotation = this->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
//        if (this->lastRangeReadings[sensor_index] < this->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {
//
//            double opposite = argos::Sin(sensor_rotation) * this->lastRangeReadings[sensor_index];
//            double adjacent = argos::Cos(sensor_rotation) * this->lastRangeReadings[sensor_index];
//
//
//            Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
//            addFreeAreaBetween(this->position, object);
//            //If the detected object is actually another agent, add it as a free area
//            //So check if the object coordinate is close to another agent
//            bool close_to_other_agent = false;
//            for (const auto &agentLocation: this->agentLocations) {
//                argos::CVector2 objectToAgent =
//                        argos::CVector2(agentLocation.second.first.x, agentLocation.second.first.y)
//                        - argos::CVector2(object.x, object.y);
//
//                //If detected object and another agent are not close, add the object as an obstacle
//                if (objectToAgent.Length() <= this->coverageMatrix->getResolution()) {
//                    close_to_other_agent = true;
//                }
//            }
//            //Only add the object as an obstacle if it is not close to another agent
//            if (!close_to_other_agent) addObjectLocation(object);
//
//
//        } else {
//            double opposite = argos::Sin(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
//            double adjacent = argos::Cos(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
//
//
//            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
//            addFreeAreaBetween(this->position, end_of_ray);
//        }
//    }

    bool addedObjectAtAgentLocation = false;
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        if (this->distance_sensors[sensor_index].getDistance() < this->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {

            float sensor_probability = HC_SR04::getProbability(this->distance_sensors[sensor_index].getDistance());

            double opposite = argos::Sin(sensor_rotation) * this->distance_sensors[sensor_index].getDistance();
            double adjacent = argos::Cos(sensor_rotation) * this->distance_sensors[sensor_index].getDistance();


            Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
            bool close_to_other_agent = false;
            for (const auto &agentLocation: this->agentLocations) {
                if ((std::get<1>(agentLocation.second) - this->elapsed_ticks) / this->ticks_per_second >
                    this->config.AGENT_LOCATION_RELEVANT_S)
                    continue;
                argos::CVector2 objectToAgent =
                        argos::CVector2(std::get<0>(agentLocation.second).x, std::get<0>(agentLocation.second).y)
                        - argos::CVector2(object.x, object.y);

                //If detected object and another agent are not close, add the object as an obstacle
                if (objectToAgent.Length() <=
                        #ifdef USING_CONFIDENCE_TREE
                this->quadtree->getResolution()
                        #else
                this->coverageMatrix->getResolution()
                        #endif
                ) {
                    close_to_other_agent = true; //TODO: Due to confidence, can maybe omit this check
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if (!close_to_other_agent) {
                if (sqrt(pow(this->position.x - object.x, 2) + pow(this->position.y - object.y, 2)) <
                    this->quadtree->getResolution()) {
                    addedObjectAtAgentLocation = true;
                }
                #ifdef USING_CONFIDENCE_TREE
                quadtree::Box objectBox = addObjectLocation(object, sensor_probability);
                if (objectBox.size != 0) {
                    if (!addedObjectAtAgentLocation)
                        addFreeAreaBetween(this->position, object, objectBox, sensor_probability);
                } else {
                    if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object, objectBox, sensor_probability);
                }
                #else
                addObjectLocation(object);
                    if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object);
                #endif
            } else {
                #ifdef USING_CONFIDENCE_TREE
                if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object, sensor_probability);
                #else
                if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object);
                #endif
            }


        } else {
            double opposite = argos::Sin(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;


            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
            #ifdef USING_CONFIDENCE_TREE
            if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, end_of_ray, 1.0);
            #else
            addFreeAreaBetween(this->position, end_of_ray);
            #endif
        }
    }
}



/** Calculate the vector to avoid the virtual walls
 * If the agent is close to the border, create a vector pointing away from the border
 * Implemented according to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * However, the vector directions are flipped compared to the paper, as the paper uses a different coordinate system
 * @return a vector pointing away from the border
 */
argos::CVector2 Agent::getVirtualWallAvoidanceVector() const {
    //If the agent is close to the border, create a vector pointing away from the border
    argos::CVector2 virtualWallAvoidanceVector = {0, 0};

    if (this->position.x < this->left_right_borders.x) {
        virtualWallAvoidanceVector.SetX(1);
    } else if (this->left_right_borders.x <= this->position.x && this->position.x <= this->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(0);
    } else if (this->position.x > this->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(-1);
    }

    if (this->position.y < this->upper_lower_borders.y) {
        virtualWallAvoidanceVector.SetY(1);
    } else if (this->upper_lower_borders.y <= this->position.y && this->position.y <= this->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(0);
    } else if (this->position.y > this->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(-1);
    }


    return virtualWallAvoidanceVector;

}

bool Agent::getAverageNeighborLocation(Coordinate *averageNeighborLocation, double range) {
    int nAgentsWithinRange = 0;
    for (const auto &agentLocation: this->agentLocations) {
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(agentLocation.second.first.x, agentLocation.second.first.y)
                - argos::CVector2(this->position.x, this->position.y);

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

argos::CVector2 Agent::calculateAgentCohesionVector() {
    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(&averageNeighborLocation, this->config.AGENT_COHESION_RADIUS);
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
argos::CVector2 Agent::calculateAgentAvoidanceVector() {

    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(&averageNeighborLocation, this->config.AGENT_AVOIDANCE_RADIUS);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between this agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(this->position.x, this->position.y);

    return vectorToOtherAgents * -1;
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


    //Get the velocities of the agents within range
    for (const auto &agentVelocity: agentVelocities) {
        std::string agentID = agentVelocity.first;
        Coordinate otherAgentLocation = agentLocations[agentID].first;
        argos::CVector2 agentVector = agentVelocity.second.first;
        double agentSpeed = agentVelocity.second.second;
        argos::CVector2 vectorToOtherAgent = argos::CVector2(otherAgentLocation.x, otherAgentLocation.y)
                                             - argos::CVector2(this->position.x, this->position.y);
        if (vectorToOtherAgent.Length() < this->config.AGENT_ALIGNMENT_RADIUS) {
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

class FrontierMerger {

private:
    // Directions for adjacency (up, down, left, right)
    const std::vector<std::pair<int, int>> directions = {{-1, 0},
                                                         {1,  0},
                                                         {0,  -1},
                                                         {0,  1}};

// Check if a cell is within bounds
    bool isValid(int x, int y, int X, int Y, const std::set<std::pair<int, int>> &cells,
                 std::set<std::pair<int, int>> &visited) {
        return x >= 0 && x < X && y >= 0 && y < Y && cells.count({x, y}) && !visited.count({x, y});
    }

public:
// Perform BFS to find a connected group
    std::vector<std::pair<int, int>> bfs_group(
            int startX, int startY, int X, int Y,
            const std::set<std::pair<int, int>> &cells,
            std::set<std::pair<int, int>> &visited) {
        std::vector<std::pair<int, int>> group;
        std::queue<std::pair<int, int>> q;

        q.push({startX, startY});
        visited.insert({startX, startY});

        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();
            group.push_back({x, y});

            for (const auto &dir: directions) {
                int nx = x + dir.first;
                int ny = y + dir.second;

                if (isValid(nx, ny, X, Y, cells, visited)) {
                    q.push({nx, ny});
                    visited.insert({nx, ny});
                }
            }
        }

        return group;
    }
};

#ifndef USING_CONFIDENCE_TREE

//ONLY FOR COVERAGE MATRIX
//A cell is a frontier iff:
//1. Occupancy = explored, i.e. pheromone > 0
//2. At least one neighbor is unexplored (i.e. pheromone == 0) using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

//Global definition of grid occupancy:
//0: unexplored
//1: explored (and free)
//2: obstacle

//Global definition of frontier cell:
//A cell is a frontier iff:
//1. Occupancy  == 1
//2. There is a 8 Moore neighbor, of which occupancy == 0

std::vector<std::pair<int, int>> Agent::getFrontierCells(double currentTimeS, double searchRadius) {
    std::vector<std::pair<int, int>> frontierCells;

    //Search within the search radius
    auto i_min = std::max(0, this->coverageMatrix->getIndexFromRealCoordinate({this->position.x - searchRadius, 0}).first);
    auto i_max = std::min(this->coverageMatrix->getWidth(), this->coverageMatrix->getIndexFromRealCoordinate({this->position.x + searchRadius, 0}).first);
    auto j_min = std::max(0, this->coverageMatrix->getIndexFromRealCoordinate({0, this->position.y - searchRadius}).second);
    auto j_max = std::min(this->coverageMatrix->getHeight(), this->coverageMatrix->getIndexFromRealCoordinate({0, this->position.y + searchRadius}).second);


    for (int i = i_min; i < i_max; i++) {
        for (int j = j_min; j < j_max; j++) {
            if(this->coverageMatrix->getByIndex(i, j, currentTimeS) > 0){ //Check if the cell is explored
                Coordinate realCoordinate = this->coverageMatrix->getRealCoordinateFromIndex(i, j);
                auto [i_obstacle, j_obstacle] = this->obstacleMatrix->getIndexFromRealCoordinate(realCoordinate);
                //Check if at least one neighbor is unexplored

                std::array<double, 9> coverageMatrixNeighbors = this->coverageMatrix->MooreNeighbors(i, j, currentTimeS);
                std::array<double, 9> obstacleMatrixNeighbors = this->obstacleMatrix->MooreNeighbors(i_obstacle, j_obstacle, currentTimeS);

                for (int k = 0; k < 9; k++) {
                    if (k == 4) continue; //Skip the middle cell
                    if (coverageMatrixNeighbors[k] == 0 && obstacleMatrixNeighbors[k] == 0) {
                        frontierCells.emplace_back(i, j); //At least one neighbor is unexplored (not free, and not an obstacle)
                        break;
                    }
                }
            }
        }
    }

    return frontierCells;
}
#endif

//argos::CVector2 Agent::calculateUnexploredFrontierVector() {
//    //According to Dynamic frontier-led swarming:
//    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
//    //F* = arg min (Ψ_D||p-G^f||_2 - Ψ_S * J^f) for all frontiers F^f in F
//    //F^f is a frontier, a segment that separates explored cells from unexplored cells.
//
//    //Where Ψ_D is the frontier distance weight
//    //p is the agent position
//    //G^f is the frontier position defined as G^f = (Sum(F_j^f))/J^f So the sum of the cell locations divided by the amount of cells
//    //Ψ_S is the frontier size weight
//    //J^f is the number of cells in the frontier F^f
//
//    //A cell is a frontier iff:
//    //1. Occupancy = explored (and free)
//    //2. At least one neighbor is unexplored using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)
//    //Coverage status: 0 for unexplored, 1 for explored, and 2 for obstacle, respectively.
//
//    //TODO: Need to keep search area small for computation times. Maybe when in range only low scores, expand range or search a box besides.
////    std::vector<quadtree::Box> frontiers = this->quadtree->queryFrontierBoxes(this->position, FRONTIER_SEARCH_RADIUS,
////                                                                              this->elapsed_ticks /
////                                                                              this->ticks_per_second);
//    const std::vector<std::pair<int, int>> frontiers = getFrontierCells(elapsed_ticks / ticks_per_second, this->config.FRONTIER_SEARCH_RADIUS);
////    current_frontiers = frontiers;
//
//    // Initialize an empty vector of vectors to store frontier regions
////    std::vector<std::vector<Coordinate>> frontierRegions = {};
//
//    std::vector<std::vector<std::pair<int, int>>> frontierRegions = {};
//    std::set<std::pair<int, int>> visited = {};
//
//    std::set<std::pair<int, int>> frontierset(frontiers.begin(), frontiers.end());
//
//    FrontierMerger frontierMerger;
//    for (const auto& cell : frontiers) {
//        if (!visited.count(cell)) {
//            // Start BFS for this cell
//            std::vector<std::pair<int, int>> group = frontierMerger.bfs_group(cell.first, cell.second, this->coverageMatrix->getWidth(), this->coverageMatrix->getHeight(), frontierset, visited);
//            frontierRegions.push_back(group);
//        }
//    }
//
//    current_frontier_regions = frontierRegions;
//
//    //Now we have all frontier cells merged into frontier regions
//    //Find F* by using the formula above
//    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
//    //Ψ_S = FRONTIER_SIZE_WEIGHT
//
//    //Initialize variables to store the best frontier region and its score
//    Coordinate bestFrontierRegionCenter = {MAXFLOAT, MAXFLOAT};
//    double highestFrontierFitness = -std::numeric_limits<double>::max();
//
//    //Iterate over all frontier regions to find the best one
//    for (const auto &region: frontierRegions) {
//        //Calculate the average position of the frontier region
//        double sumX = 0;
//        double sumY = 0;
//        for (auto cellIndex: region) {
//            auto realCellCoordinate = this->coverageMatrix->getRealCoordinateFromIndex(cellIndex.first, cellIndex.second);
//            sumX += realCellCoordinate.x;
//            sumY += realCellCoordinate.y;
//        }
//        double frontierRegionX = sumX / region.size();
//        double frontierRegionY = sumY / region.size();
//
//        //Calculate the distance between the agent and the frontier region
//        double distance = sqrt(pow(frontierRegionX - this->position.x, 2) + pow(frontierRegionY - this->position.y, 2));
//        //If the frontier location is too close to the current position, disregard it as that area is explored already.
////        if(distance < 0.5){
////            continue;
////        }
//
//        //Calculate the fitness of the frontier region
//        double fitness = -this->config.FRONTIER_DISTANCE_WEIGHT * distance + this->config.FRONTIER_SIZE_WEIGHT * region.size();
//
//        //If the fitness is lower than the best fitness, update the best fitness and best frontier region
//        if (fitness > highestFrontierFitness) {
//            highestFrontierFitness = fitness;
//            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
//        }
//    }
//
//    this->currentBestFrontier = bestFrontierRegionCenter;
//
//    //Own fix:
//    //If there is no best frontier region, return a zero vector
//    if (bestFrontierRegionCenter.x == MAXFLOAT) {
//        return {0, 0};
//    }
//
//
//
//
//    //Calculate the vector to the best frontier region
//    argos::CVector2 vectorToBestFrontier = argos::CVector2(bestFrontierRegionCenter.x, bestFrontierRegionCenter.y)
//                                           - argos::CVector2(this->position.x, this->position.y);
//
//    return vectorToBestFrontier;
//}

void Agent::calculateNextPosition() {
    //Inspired by boids algorithm:
    //Vector determining heading
    //Vector is composed of:
    //1. Attraction to unexplored frontier
    //2. Repulsion from other agents (done)
    //3. Attraction to found target
    //4. Repulsion from objects/walls (done)


    argos::CVector2 virtualWallAvoidanceVector = ForceVectorCalculator::getVirtualWallAvoidanceVector(this);
    argos::CVector2 agentCohesionVector = ForceVectorCalculator::calculateAgentCohesionVector(this);
    argos::CVector2 agentAvoidanceVector = ForceVectorCalculator::calculateAgentAvoidanceVector(this);
    argos::CVector2 agentAlignmentVector = ForceVectorCalculator::calculateAgentAlignmentVector(this);

    argos::CVector2 unexploredFrontierVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);

    //If there are agents to avoid, do not explore
    if (agentAvoidanceVector.Length() != 0) unexploredFrontierVector = {0, 0};


    //Normalize vectors if they are not zero
    if (virtualWallAvoidanceVector.Length() != 0) virtualWallAvoidanceVector.Normalize();
    if (agentCohesionVector.Length() != 0) agentCohesionVector.Normalize();
    if (agentAvoidanceVector.Length() != 0) agentAvoidanceVector.Normalize();
    if (agentAlignmentVector.Length() != 0) agentAlignmentVector.Normalize();
    if (unexploredFrontierVector.Length() != 0) unexploredFrontierVector.Normalize();

    virtualWallAvoidanceVector = this->config.VIRTUAL_WALL_AVOIDANCE_WEIGHT * virtualWallAvoidanceVector;
    agentCohesionVector = this->config.AGENT_COHESION_WEIGHT * agentCohesionVector; //Normalize first
    agentAvoidanceVector = this->config.AGENT_AVOIDANCE_WEIGHT * agentAvoidanceVector;
    agentAlignmentVector = this->config.AGENT_ALIGNMENT_WEIGHT * agentAlignmentVector;
    unexploredFrontierVector = this->config.TARGET_WEIGHT * unexploredFrontierVector;

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
    argos::CRadians objectAvoidanceAngle;
    bool isThereAFreeAngle = ForceVectorCalculator::calculateObjectAvoidanceAngle(&objectAvoidanceAngle, this->swarm_vector.Angle());
    //If there is not a free angle to move to, do not move
    if (!isThereAFreeAngle) {
        this->force_vector = {0, 0};
    } else {


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
// Edit: contacted the writer, he said below is correct
        this->force_vector = {(this->swarm_vector.GetX()) *
                              argos::Cos(objectAvoidanceAngle) -
                              (this->swarm_vector.GetY()) *
                              argos::Sin(objectAvoidanceAngle),
                              (this->swarm_vector.GetX()) *
                              argos::Sin(objectAvoidanceAngle) +
                              (this->swarm_vector.GetY()) *
                              argos::Cos(objectAvoidanceAngle)};

        argos::CRadians angle = this->force_vector.Angle();
        this->targetHeading = angle;
    }
}

void Agent::calculateNextPosition() {
    //Inspired by boids algorithm:
    //Vector determining heading
    //Vector is composed of:
    //1. Attraction to unexplored frontier
    //2. Repulsion from other agents (done)
    //3. Attraction to found target
    //4. Repulsion from objects/walls (done)


    argos::CVector2 virtualWallAvoidanceVector = ForceVectorCalculator::getVirtualWallAvoidanceVector(this);
    argos::CVector2 agentCohesionVector = ForceVectorCalculator::calculateAgentCohesionVector(this);
    argos::CVector2 agentAvoidanceVector = ForceVectorCalculator::calculateAgentAvoidanceVector(this);
    argos::CVector2 agentAlignmentVector = ForceVectorCalculator::calculateAgentAlignmentVector(this);

    argos::CVector2 targetVector = argos::CVector2(this->currentBestFrontier.x - this->position.x,
                                                   this->currentBestFrontier.y - this->position.y);


    if (this->state == State::EXPLORING) {





        //Find new frontier
        targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
        bool noTarget = this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT};

        ForceVectorCalculator::vectors vectors{this->swarm_vector, virtualWallAvoidanceVector, agentCohesionVector,
                                               agentAvoidanceVector, agentAlignmentVector, targetVector};

        ForceVectorCalculator::checkAvoidAndNormalizeVectors(vectors);

        argos::CVector2 total_vector;
        argos::CRadians objectAvoidanceAngle;
        bool isThereAFreeAngle = ForceVectorCalculator::calculateObjectAvoidanceAngle(this, &objectAvoidanceAngle,
                                                                                      vectors,
                                                                                      total_vector,
                                                                                      false);//targetVector.Length() == 0);
        //If there is not a free angle to move to, do not move
        if (!isThereAFreeAngle) {
            this->force_vector = {0, 0};
        } else {


// According to the paper, the formula should be:
//        this->force_vector = {(this->previous_total_vector.GetX()) *
//                              argos::Cos(objectAvoidanceAngle) -
//                              (this->previous_total_vector.GetX()) *
//                              argos::Sin(objectAvoidanceAngle),
//                              (this->previous_total_vector.GetY()) *
//                              argos::Sin(objectAvoidanceAngle) +
//                              (this->previous_total_vector.GetY()) *
//                              argos::Cos(objectAvoidanceAngle)};

// However, according to theory (https://en.wikipedia.org/wiki/Rotation_matrix) (https://www.purplemath.com/modules/idents.htm), the formula should be:
// This also seems to give better performance.
// Edit: contacted the writer, he said below is correct
            this->force_vector = {(total_vector.GetX()) *
                                  argos::Cos(objectAvoidanceAngle) -
                                  (total_vector.GetY()) *
                                  argos::Sin(objectAvoidanceAngle),
                                  (total_vector.GetX()) *
                                  argos::Sin(objectAvoidanceAngle) +
                                  (total_vector.GetY()) *
                                  argos::Cos(objectAvoidanceAngle)};

            argos::CRadians angle = this->force_vector.Angle();
            this->targetHeading = angle;
        }
        this->swarm_vector = total_vector;
    }
}

void Agent::timeSyncWithCloseAgents() {
    for (const auto& agentLocationPair: this->agentLocations) {
        double lastReceivedTick = agentLocationPair.second.second;
        //If we have received the location of this agent in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds (so it is probably within communication range), broadcast time sync init
        if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second <
            this->config.AGENT_LOCATION_RELEVANT_S) {
            //If we have not time synced with this agent in the past TIME_SYNC_INTERVAL_S seconds, do it
            if (this->elapsed_ticks - this->timeSynchronizer.getLastSync(agentLocationPair.first) >= this->config.TIME_SYNC_INTERVAL_S * this->ticks_per_second) {
                timeSynchronizer.initTimeSync(this); //Broadcasting time sync init
            }
        }
    }
}

void Agent::doStep() {
    broadcastMessage("C:" + this->position.toString());
    std::string coverageMatrixString = this->coverageMatrix->matrixToString();
    broadcastMessage("MC:" + coverageMatrixString);
    std::string obstacleMatrixString = this->obstacleMatrix->matrixToString();
    broadcastMessage("MO:" + obstacleMatrixString);
    timeSyncWithCloseAgents();

    broadcastMessage(
            "V:" + std::to_string(this->force_vector.GetX()) + ";" + std::to_string(this->force_vector.GetY()) +
            ":" + std::to_string(this->speed));

    checkMessages();

    checkForObstacles();

    calculateNextPosition();

    //If there is no force vector, do not move
    if (this->force_vector == argos::CVector2{0, 0}) this->differential_drive.stop();
    else {

        argos::CRadians diff = (this->heading - this->targetHeading).SignedNormalize();

        argos::CDegrees diffDeg = ToDegrees(diff);


        if (diffDeg > argos::CDegrees(-this->config.TURN_THRESHOLD_DEGREES) && diffDeg < argos::CDegrees(this->config.TURN_THRESHOLD_DEGREES)) {
            //Go straight
            this->differential_drive.forward();
        } else if (diffDeg > argos::CDegrees(0)) {
            //turn right
            this->differential_drive.turnRight();
        } else {
            //turn left
            this->differential_drive.turnLeft();
        }
    }


    this->elapsed_ticks++;
}


/**
 * Broadcast a message to all agents
 * @param message
 */
void Agent::broadcastMessage(const std::string &message) const {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    this->wifi.broadcast_message(messagePrependedWithId);
}

/**
 * Sends a message to an agents
 * @param message
 */
void Agent::sendMessage(const std::string &message, const std::string& targetId) {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    this->wifi.send_message(messagePrependedWithId, targetId);
}



/**
 * Check for messages from other agents
 * If there are messages, parse them
 */
void Agent::checkMessages() {
    //Read messages from other agents
    this->wifi.receive_messages(this->messages, this->elapsed_ticks/this->ticks_per_second);
    if (!this->messages.empty()) parseMessages();

}

/**
 * Get the target id from a message
 * @param message
 * @return
 */
std::string getTargetIdFromMessage(const std::string &message) {
    return message.substr(message.find('<')+1, message.find('>') - 1 - message.find('<'));

}

/**
 * Get the id from a message
 * @param message
 * @return
 */
std::string getIdFromMessage(const std::string &message) {
    return message.substr(message.find('[')+1, message.find(']') - 1 - message.find('['));

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
    Coordinate newCoordinate{};
    newCoordinate.x = std::stod(token);
    str.erase(0, pos + delimiter.length());
    newCoordinate.y = std::stod(str);
    return newCoordinate;
}

///**
// * Get a QuadNode from a string
// * @param str
// * @return
// */
//quadtree::QuadNode quadNodeFromString(std::string str) {
//    std::string visitedDelimiter = "@";
//    std::string occDelimiter = ":";
//    size_t visitedPos = 0;
//    size_t occPos = 0;
//    std::string coordinate;
//    std::string occ;
//    std::string visited;
//    occPos = str.find(occDelimiter);
//    coordinate = str.substr(0, occPos);
//    quadtree::QuadNode newQuadNode{};
//    newQuadNode.coordinate = coordinateFromString(coordinate);
//    str.erase(0, occPos + occDelimiter.length());
//    //Now we have the occupancy and ticks
//
//    visitedPos = str.find(visitedDelimiter);
//    occ = str.substr(0, visitedPos);
//    newQuadNode.occupancy = static_cast<quadtree::Occupancy>(std::stoi(occ));
//    str.erase(0, visitedPos + visitedDelimiter.length());
//    visited = str;
//    newQuadNode.visitedAtS = std::stod(visited);
//    return newQuadNode;
//}


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

std::vector<std::string> splitString(const std::string &str, const std::string &delimiter) {
    std::vector<std::string> strings;
    size_t pos = 0;
    size_t lastPos = 0;
    while ((pos = str.find(delimiter, lastPos)) != std::string::npos) {
        strings.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = pos + delimiter.length();
    }
    strings.push_back(str.substr(lastPos, str.length() - lastPos));
    return strings;
}


/**
 * Parse messages from other agents
 */
void Agent::parseMessages() {
    for (const std::string &message: this->messages) {
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent.at(0) == 'C') {
            Coordinate receivedPosition = coordinateFromString(messageContent.substr(2));
            this->agentLocations[senderId] = {receivedPosition, this->elapsed_ticks};
        } else if (messageContent.at(0) == 'M') {
            auto substring = messageContent.substr(1);
            auto coverage = substring.at(0) == 'C';
            std::vector<std::string> chunks;
            std::stringstream ss(messageContent.substr(3));
            std::string chunk;
            int i = 0;
            while (std::getline(ss, chunk, '|')) {
                std::stringstream chunkStream(chunk);
                std::string cellStr;
                int j = 0;
                while(std::getline(chunkStream, cellStr, ';')){
                    if (coverage) {
                        this->coverageMatrix->updateByIndex(i, j, std::stod(cellStr));
                    } else {
                        auto value = std::stod(cellStr);
                        this->obstacleMatrix->updateByIndex(i, j, value);
                        //If we have an obstacle, reset the corresponding coverage matrix cell
                        if (value != -1){
                            auto realObstacleCoordinate = this->obstacleMatrix->getRealCoordinateFromIndex(i, j);
                            this->coverageMatrix->reset(realObstacleCoordinate);
                        }
                    }
                    j++;
                }
                i++;


            }

        } else if (messageContent.at(0) == 'V') {
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
        } else if (messageContent[0] == 'T') { //Time sync message
            std::string timeSyncString = messageContent.substr(2);
            auto splitStrings = splitString(timeSyncString, "|");
            int messageType = std::stoi(splitStrings[0]);
            switch (messageType) {
                case 0: {
                    int t_TXi = std::stoi(splitStrings[1]);
                    timeSynchronizer.respondToTimeSync(senderId, this, t_TXi);
                    break;
                }
                case 1: {
                    int t_TXi = std::stoi(splitStrings[1]);
                    int t_RXj = std::stoi(splitStrings[2]);
                    int t_TXj = std::stoi(splitStrings[3]);
                    timeSynchronizer.determineT_RXi(senderId, this, t_TXi, t_RXj, t_TXj);
                    break;
                }
                case 2: {
                    int t_RXi = std::stoi(splitStrings[1]);
                    timeSynchronizer.receiveT_RXi(senderId, this, t_RXi);
                    break;
                }
                default : {
                    assert(0 && "Unknown time sync message type");
                }
            }

        }
    }

}


Radio Agent::getWifi() const {
    return this->wifi;
}

void Agent::setWifi(Radio newWifi) {
    this->wifi = newWifi;
    this->wifi.config(this->config.WIFI_SPEED_MBPS, this->config.MAX_JITTER_MS, this->config.MESSAGE_LOSS_PROBABILITY);

}

std::vector<std::string> Agent::getMessages() {
    return this->messages;
}

void Agent::loadConfig() {
    YAML::Node config_yaml = YAML::LoadFile(config_file);

    this->config.ROBOT_WEIGHT = config_yaml["physical"]["robot_weight"].as<float>();
    this->config.ROBOT_WHEEL_RADIUS = config_yaml["physical"]["robot_wheel_radius"].as<float>();
    this->config.ROBOT_INTER_WHEEL_DISTANCE = config_yaml["physical"]["robot_inter_wheel_distance"].as<float>();
//
    this->config.TURN_THRESHOLD_DEGREES = config_yaml["control"]["turn_threshold"].as<double>();
    this->config.TURNING_SPEED_RATIO = config_yaml["control"]["turn_speed_ratio"].as<float>();
    this->config.STEPS_360_DEGREES = config_yaml["control"]["360_degrees_steps"].as<double>();
    this->config.AGENT_SAFETY_RADIUS = config_yaml["physical"]["robot_diameter"].as<double>() + config_yaml["control"]["agent_safety_radius_margin"].as<double>();
    this->config.OBJECT_SAFETY_RADIUS = config_yaml["control"]["object_safety_radius"].as<double>();
//    this->config.FRONTIER_DIST_UNTIL_REACHED = config_yaml["control"]["disallow_frontier_switching"]["frontier_reach_distance"].as<double>();
//#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
//    this->config.PERIODIC_FEASIBILITY_CHECK_INTERVAL_S = config_yaml["control"]["disallow_frontier_switching"]["target_feasibility_check_interval"].as<float>();
//    this->config.FEASIBILITY_CHECK_ONLY_ROUTE = config_yaml["control"]["disallow_frontier_switching"]["feasibility_check_only_route"].as<bool>();
//#endif
//#ifdef SEPARATE_FRONTIERS
//    this->config.FRONTIER_SEPARATION_THRESHOLD = config_yaml["control"]["separate_frontiers"]["distance_threshold"].as<float>();
//#endif
    this->config.AGENT_LOCATION_RELEVANT_S = config_yaml["communication"]["agent_info_relevant"].as<double>();
    this->config.MAP_EXCHANGE_INTERVAL_S = config_yaml["communication"]["matrix_exchange_interval"].as<double>();
    this->config.TIME_SYNC_INTERVAL_S = config_yaml["communication"]["time_sync_interval"].as<double>();
//
    this->config.ORIENTATION_NOISE_DEGREES = config_yaml["sensors"]["orientation_noise"].as<double>();
    this->config.ORIENTATION_JITTER_DEGREES = config_yaml["sensors"]["orientation_jitter"].as<double>();
    this->config.DISTANCE_SENSOR_NOISE_CM = config_yaml["sensors"]["distance_sensor_noise"].as<double>();
    this->config.POSITION_NOISE_CM = config_yaml["sensors"]["position_noise"].as<double>();
    this->config.POSITION_JITTER_CM = config_yaml["sensors"]["position_jitter"].as<double>();
    this->config.DISTANCE_SENSOR_PROXIMITY_RANGE = config_yaml["sensors"]["distance_sensor_range"].as<double>();
////
    this->config.FRONTIER_SEARCH_RADIUS = config_yaml["forces"]["frontier_search_radius"].as<double>();
    this->config.MAX_FRONTIER_CELLS = config_yaml["forces"]["max_frontier_cells"].as<int>();
    this->config.MAX_FRONTIER_REGIONS = config_yaml["forces"]["max_frontier_regions"].as<int>();
    this->config.AGENT_AVOIDANCE_RADIUS = config_yaml["forces"]["agent_avoidance_radius"].as<double>();
    this->config.AGENT_COHESION_RADIUS = config_yaml["forces"]["agent_cohesion_radius"].as<double>();
    this->config.AGENT_ALIGNMENT_RADIUS = config_yaml["forces"]["agent_alignment_radius"].as<double>();
    this->config.OBJECT_AVOIDANCE_RADIUS = this->config.AGENT_SAFETY_RADIUS + this->config.OBJECT_SAFETY_RADIUS + config_yaml["forces"]["object_avoidance_radius_margin"].as<double>();
//
    this->config.VIRTUAL_WALL_AVOIDANCE_WEIGHT = config_yaml["forces"]["virtual_wall_avoidance_weight"].as<double>();
    this->config.AGENT_COHESION_WEIGHT = config_yaml["forces"]["agent_cohesion_weight"].as<double>();
    this->config.AGENT_AVOIDANCE_WEIGHT = config_yaml["forces"]["agent_avoidance_weight"].as<double>();
    this->config.AGENT_ALIGNMENT_WEIGHT = config_yaml["forces"]["agent_alignment_weight"].as<double>();
    this->config.TARGET_WEIGHT = config_yaml["forces"]["unexplored_frontier_weight"].as<double>();
//
    this->config.FRONTIER_DISTANCE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["distance_weight"].as<double>();
    this->config.FRONTIER_SIZE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["size_weight"].as<double>();
//    this->config.FRONTIER_REACH_BATTERY_WEIGHT = config_yaml["forces"]["frontier_fitness"]["reach_battery_weight"].as<double>();
//    this->config.FRONTIER_REACH_DURATION_WEIGHT = config_yaml["forces"]["frontier_fitness"]["reach_duration_weight"].as<double>();
//    this->config.FRONTIER_PHEROMONE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["pheromone_weight"].as<double>();

    #ifdef USING_CONFIDENCE_TREE
    this->config.P_FREE = config_yaml["confidence"]["p_free"].as<double>();
    this->config.P_OCCUPIED = config_yaml["confidence"]["p_occupied"].as<double>();
    this->config.ALPHA_RECEIVE = config_yaml["confidence"]["alpha_receive"].as<float>();
    this->config.P_FREE_THRESHOLD = config_yaml["confidence"]["p_free_threshold"].as<float>();
    this->config.P_OCCUPIED_THRESHOLD = config_yaml["confidence"]["p_free_threshold"].as<float>();
    this->config.P_AT_MAX_SENSOR_RANGE = config_yaml["confidence"]["p_at_max_sensor_range"].as<float>();

    this->config.QUADTREE_RESOLUTION = config_yaml["quadtree"]["resolution"].as<double>();
    this->config.QUADTREE_EVAPORATION_TIME_S = config_yaml["quadtree"]["evaporation_time"].as<double>();
    this->config.QUADTREE_EVAPORATED_PHEROMONE_FACTOR = config_yaml["quadtree"]["evaporated_pheromone_factor"].as<double>();
    this->config.QUADTREE_MERGE_MAX_VISITED_TIME_DIFF = config_yaml["quadtree"]["merge_max_visited_time_difference"].as<double>();
    this->config.QUADTREE_MERGE_MAX_P_CONFIDENCE_DIFF = config_yaml["quadtree"]["merge_max_confidence_diff"].as<double>();
    #else
    this->config.COVERAGE_MATRIX_RESOLUTION = config_yaml["map"]["coverage_resolution"].as<double>();
    this->config.OBSTACLE_MATRIX_RESOLUTION = config_yaml["map"]["obstacle_resolution"].as<double>();
    this->config.COVERAGE_MATRIX_EVAPORATION_TIME_S = config_yaml["map"]["coverage_evaporation_time"].as<double>();
    this->config.OBSTACLE_MATRIX_EVAPORATION_TIME_S = config_yaml["map"]["obstacle_evaporation_time"].as<double>();
    #endif
//    this->config.QUADTREE_EVAPORATION_TIME_S = config_yaml["quadtree"]["evaporation_time"].as<double>();
//    this->config.QUADTREE_EVAPORATED_PHEROMONE_FACTOR = config_yaml["quadtree"]["evaporated_pheromone_factor"].as<double>();
//    this->config.QUADTREE_MERGE_MAX_VISITED_TIME_DIFF = config_yaml["quadtree"]["merge_max_visited_time_difference"].as<double>();
//    this->config.QUADTREE_MERGE_MAX_P_CONFIDENCE_DIFF = config_yaml["quadtree"]["merge_max_confidence_diff"].as<double>();
//
    this->config.BATTERY_CAPACITY = config_yaml["battery"]["capacity"].as<double>();
    this->config.BATTERY_VOLTAGE = config_yaml["battery"]["voltage"].as<double>();
//
    this->config.MOTOR_STALL_CURRENT = config_yaml["motor"]["stall_current"].as<double>();
    this->config.MOTOR_STALL_TORQUE = config_yaml["motor"]["stall_torque"].as<double>();
    this->config.MOTOR_NO_LOAD_RPM = config_yaml["motor"]["no_load_rpm"].as<double>();
    this->config.MOTOR_NO_LOAD_CURRENT = config_yaml["motor"]["no_load_current"].as<double>();
//
    this->config.WIFI_SPEED_MBPS = config_yaml["communication"]["wifi_speed"].as<double>();
    this->config.MAX_JITTER_MS = config_yaml["communication"]["max_jitter"].as<double>();
    this->config.MESSAGE_LOSS_PROBABILITY = config_yaml["communication"]["message_loss_probability"].as<double>();
}