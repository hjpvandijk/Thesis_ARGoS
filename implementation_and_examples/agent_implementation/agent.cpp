//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include <set>
#include "agent.h"
#include "agent_control/path_planning/WallFollower.h"
#include <random>
#include <utility>
#include "utils/CustomComparator.h"


Agent::Agent(std::string id) {
    this->id = std::move(id);
    this->position = {0.0, 0.0};
    this->heading = argos::CRadians(0);
    this->targetHeading = argos::CRadians(0);
    this->speed = 1;
    this->swarm_vector = argos::CVector2(0, 0);
    this->force_vector = argos::CVector2(0, 1);
    this->messages = std::vector<std::string>(0);
    auto box = quadtree::Box(-5, 5, 10);
    this->quadtree = std::make_unique<quadtree::Quadtree>(box);
    this->wallFollower = WallFollower();

#ifdef BATTERY_MANAGEMENT_ENABLED
    //TODO: Get values from config file
    //https://e-puck.gctronic.com/index.php?option=com_content&view=article&id=7&Itemid=9
    //Motor stall values based on the tt dc gearbox motor (https://www.sgbotic.com/index.php?dispatch=products.view&product_id=2674)
    this->batteryManager = BatteryManager(0.4f, 0.0205f, 0.0565f, 0.8, 250, 1.5, 0.16, 6, 1000);
    //Set the speed to the maximum achievable speed, based on the the motor specs. TODO: Put that info in differential drive instead
    auto max_achievable_speed = this->batteryManager.motionSystemBatteryManager.getMaxAchievableSpeed();
    this->differential_drive = DifferentialDrive(std::min(max_achievable_speed, this->speed), std::min(max_achievable_speed, this->speed*this->TURNING_SPEED_RATIO));
    this->speed = this->differential_drive.max_speed_straight;
    //Set the voltage to the voltage required for the current speed, and corresponding values, to use in calculations.
    this->batteryManager.motionSystemBatteryManager.calculateVoltageAtSpeed(this->speed);
#else
    this->differential_drive = DifferentialDrive(this->speed, this->speed*this->TURNING_SPEED_RATIO);
#endif

#ifdef PATH_PLANNING_ENABLED
    this->pathPlanner = SimplePathPlanner();
    this->pathFollower = PathFollower();
#endif
#ifdef AVOID_UNREACHABLE_FRONTIERS
    this->frontierEvaluator = FrontierEvaluator(this->CLOSEST_COORDINATE_HIT_COUNT_BEFORE_DECREASING_CONFIDENCE, MAX_TICKS_IN_HITPOINT);
#endif
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

/**
 * Add free (unoccupied) coordinates between two coordinates and slightly occupied coordinates after the object
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetweenAndOccupiedAfter(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox, float Psensor) const {
    if(sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2)) < this->quadtree->getSmallestBoxSize()) return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
//    if (objectBox.size > 0) return;

    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getSmallestBoxSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as free
            double p = (this->P_FREE*Psensor - 0.5) * (1 - double(s) / double(nSteps)) + 0.5; //Increasingly more uncertain the further away from the agent
            this->quadtree->add(Coordinate{x, y}, p, elapsed_ticks / ticks_per_second, elapsed_ticks/ticks_per_second);
        } else {
            break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
        }
        x += stepX;
        y += stepY;
    }

    x += stepX;
    y += stepY;

    double occ_probability = this->P_OCCUPIED / Psensor;
    for(int s = 1; s <= 2; s++){ //Add slight occupied confidence to the two steps after the object
        if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as more occupied
            double p = (0.5-occ_probability) * (double(s)/double(10)) + occ_probability; //Increasingly more uncertain the further away from the agent
            this->quadtree->add(Coordinate{x, y}, p, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
        }
        x += stepX;
        y += stepY;

    }
}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox, float Psensor) const {
    if(sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2)) < this->quadtree->getSmallestBoxSize()) return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getSmallestBoxSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        if (objectBox.size > 0) {
            if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as free
                double p = (this->P_FREE - 0.5) * (1 - double(s) / double(nSteps)) + 0.5; //Increasingly more uncertain the further away from the agent
                this->quadtree->add(Coordinate{x, y}, p * Psensor, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
            } else {
                break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
            }
        }
        x += stepX;
        y += stepY;
    }
}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, float Psensor) const {
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getSmallestBoxSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        this->quadtree->add(Coordinate{x, y}, this->P_FREE * Psensor, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
        x += stepX;
        y += stepY;
    }
}

/**
 * Add occupied coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addOccupiedAreaBetween(Coordinate coordinate1, Coordinate coordinate2) const {
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getSmallestBoxSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps-1; s++) {
        this->quadtree->add(Coordinate{x, y}, this->P_OCCUPIED, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
        x += stepX;
        y += stepY;
    }
}

bool Agent::isObstacleBetween(Coordinate coordinate1, Coordinate coordinate2) const {
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x -
                0.02; // Subtract a small margin to detecting boxes the coordinates belong to
    double dy = coordinate2.y - coordinate1.y - 0.02;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = this->quadtree->getSmallestBoxSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    x += stepX * 0.1; // Add a small margin to detecting boxes the coordinates belong to
    y += stepY * 0.1; // Add a small margin to detecting boxes the coordinates belong to


    for (int s = 1; s < nSteps; s++) {
        if (this->quadtree->getOccupancyFromCoordinate(Coordinate{x, y}) == quadtree::Occupancy::OCCUPIED) {
            return true;
        }
        x += stepX;
        y += stepY;
    }
    return false;
}


/**
 * Add occupied object location to the quadtree
 * @param objectCoordinate
 */
quadtree::Box Agent::addObjectLocation(Coordinate objectCoordinate, float Psensor) const {
    quadtree::Box objectBox = this->quadtree->add(objectCoordinate, this->P_OCCUPIED / Psensor, //Divided by sensor accuracy probability (so higher resulting probability) as maybe the object is not there
                                                  elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
#ifdef CLOSE_SMALL_AREAS
    if (objectBox.getSize() != 0) // If the box is not the zero (not added)
        checkIfAgentFitsBetweenObstacles(objectBox);
#endif
    return objectBox;
}

/**
 * Check for obstacles in front of the agent
 * If there is an obstacle within a certain range, add the free area between the agent and the obstacle to the quadtree
 * If there is no obstacle within range, add the free area between the agent and the end of the range to the quadtree
 */
void Agent::checkForObstacles() {
    bool addedObjectAtAgentLocation = false;
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        if (this->distance_sensors[sensor_index].getDistance() < PROXIMITY_RANGE) {

            float sensor_probability = HC_SR04::getProbability(this->distance_sensors[sensor_index].getDistance());

            double opposite = argos::Sin(sensor_rotation) * this->distance_sensors[sensor_index].getDistance();
            double adjacent = argos::Cos(sensor_rotation) * this->distance_sensors[sensor_index].getDistance();


            Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
            bool close_to_other_agent = false;
            for (const auto &agentLocation: this->agentLocations) {
                if((std::get<2>(agentLocation.second) - this->elapsed_ticks) / this->ticks_per_second > AGENT_LOCATION_RELEVANT_DURATION_S) continue;
                argos::CVector2 objectToAgent =
                        argos::CVector2(std::get<0>(agentLocation.second).x, std::get<0>(agentLocation.second).y)
                        - argos::CVector2(object.x, object.y);

                //If detected object and another agent are not close, add the object as an obstacle
                if (objectToAgent.Length() <= this->quadtree->getSmallestBoxSize()) {
                    close_to_other_agent = true; //TODO: Due to confidence, can maybe omit this check
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if (!close_to_other_agent) {
                if (sqrt(pow(this->position.x - object.x, 2) + pow(this->position.y - object.y, 2)) < this->quadtree->getSmallestBoxSize()) {
                    addedObjectAtAgentLocation = true;
                }
                quadtree::Box objectBox = addObjectLocation(object, sensor_probability);
                if (objectBox.size != 0 ){
                    if (!addedObjectAtAgentLocation) addFreeAreaBetweenAndOccupiedAfter(this->position, object, objectBox, sensor_probability);
                } else {
                    if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object, sensor_probability);
                }
            } else {
                if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object, sensor_probability);
            }




        } else {
            double opposite = argos::Sin(sensor_rotation) * PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * PROXIMITY_RANGE;


            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
            if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, end_of_ray, 1.0);
        }
    }
}

/**
 * Check the area around the object to see if the agent fits between the object and other obstacles
 * If the agent fits, do nothing
 * If the agent does not fit, set that area as an obstacle.
 * @param objectCoordinate
 */
void Agent::checkIfAgentFitsBetweenObstacles(quadtree::Box objectBox) const {
    Coordinate objectCoordinate = objectBox.getCenter();
    std::vector<quadtree::Box> occupiedBoxes = this->quadtree->queryOccupiedBoxes(objectCoordinate,
                                                                                  3.0*OBJECT_AVOIDANCE_RADIUS,
                                                                                  this->elapsed_ticks /
                                                                                  this->ticks_per_second);
    //For each box, that is not the checked object, check if the agent fits between the object and the box
    for (auto box: occupiedBoxes) {
        if (box.contains(objectCoordinate)) {
            continue;
        }

        Coordinate objectCorner{};
        std::vector<Coordinate> objectEdges = {};
        Coordinate boxCorner{};
        std::vector<Coordinate> boxEdges = {};

        //Check which direction the object is compared to the occupied box, and check the distance between the correct corners
        auto center = box.getCenter();

        // West
        if (objectCoordinate.x < center.x) {
            //Just West
            if (objectCoordinate.y == center.y) {
                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});

                boxCorner = Coordinate{box.getBottom(), box.left};
                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
            }
                // North West
            else if (objectCoordinate.y > center.y) {
                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});

                boxCorner = Coordinate{box.top, box.left};
                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
            }
                // South West
            else if (objectCoordinate.y < center.y) {
                objectCorner = Coordinate{objectBox.top, objectBox.getRight()};
                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});

                boxCorner = Coordinate{box.getBottom(), box.left};
                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
            }
                // Not in any direction
            else {
                continue;
            }
        }
            // East
        else if (objectCoordinate.x > center.x) {
            //Just East
            if (objectCoordinate.y == center.y) {
                objectCorner = Coordinate{objectBox.getBottom(), objectBox.left};
                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});

                boxCorner = Coordinate{box.getBottom(), box.getRight()};
                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
            }
                // North East
            else if (objectCoordinate.y > center.y) {
                objectCorner = Coordinate{objectBox.getBottom(), objectBox.left};
                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});

                boxCorner = Coordinate{box.top, box.getRight()};
                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
            }
                // South East
            else if (objectCoordinate.y < center.y) {
                objectCorner = Coordinate{objectBox.top, objectBox.left};
                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});

                boxCorner = Coordinate{box.getBottom(), box.getRight()};
                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
            }
                // Not in any direction
            else {
                continue;
            }
        } else if (objectCoordinate.x == center.x) {
            //Just North
            if (objectCoordinate.y > center.y) {
                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});

                boxCorner = Coordinate{box.top, box.getRight()};
                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
            }
                //Just South
            else if (objectCoordinate.y < center.y) {
                objectCorner = Coordinate{objectBox.top, objectBox.getRight()};
                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});

                boxCorner = Coordinate{box.getBottom(), box.getRight()};
                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
            }
                // Not in any direction
            else {
                continue;
            }
        }
            // Not in any direction
        else {
            continue;
        }

        double distance = sqrt(pow(objectCorner.x - boxCorner.x, 2) + pow(objectCorner.y - boxCorner.y, 2));
        if (distance < 0.01) { // If they are adjacent (with a small margin)
            continue;
        } else if (distance <
                   this->OBJECT_AVOIDANCE_RADIUS) { // If the agent does not fit between the object and the box
            //Add the area between the object and the box as occupied if there is no occupied area between already
            bool areaFree = true;
            for (int edge_count = 0; edge_count < objectEdges.size(); edge_count++) {
                if (isObstacleBetween(objectEdges[edge_count], boxEdges[edge_count])) {
                    areaFree = false;
                }
            }
            if (areaFree) {
                addOccupiedAreaBetween(objectCoordinate, box.getCenter());
            }

        }
    }
}






bool Agent::frontierPheromoneEvaporated() {
    quadtree->queryFrontierBoxes(this->currentBestFrontier, quadtree->getSmallestBoxSize(),
                                 this->elapsed_ticks / this->ticks_per_second); //Update pheromone of frontier cell
    if (quadtree->isCoordinateUnknownOrAmbiguous(this->currentBestFrontier)) return true;
    return false;
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

    argos::CVector2 unexploredFrontierVector = argos::CVector2(this->currentBestFrontier.x - this->position.x,
                                                               this->currentBestFrontier.y - this->position.y);

#ifdef AVOID_UNREACHABLE_FRONTIERS
    frontierEvaluator.resetFrontierAvoidance(this, unexploredFrontierVector);
#endif

#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    //If the agent is close to the frontier and is heading towards it, or if it is really close to the frontier.
    //So we don't 'reach' frontiers through walls.
    bool frontierReached = unexploredFrontierVector.Length() <= FRONTIER_DIST_UNTIL_REACHED &&
                           NormalizedDifference(this->targetHeading, unexploredFrontierVector.Angle()).GetValue() <
                           this->TURN_THRESHOLD_DEGREES * 2 ||
                           unexploredFrontierVector.Length() <= this->OBJECT_AVOIDANCE_RADIUS;

    bool periodic_check_required = (this->elapsed_ticks - this->last_feasibility_check_tick) > this->ticks_per_second * this->PERIODIC_FEASIBILITY_CHECK_INTERVAL_S;

    //If the current best frontier is not set, or the agent is close to a blacklisted frontier, or the agent is close to the frontier (reached).
    if (this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT} ||
#ifdef AVOID_UNREACHABLE_FRONTIERS
    //Or if the frontier has low confidence
frontierEvaluator.frontierHasLowConfidenceOrAvoiding(this) ||
#endif
    //If the current best frontier is blacklisted
    frontierReached //|| //Or the agent is close to the frontier
//    //Or if the pheromone of cell the frontier is in has evaporated --> frontier has moved
//    frontierPheromoneEvaporated() //It can be that the frontier is not in an explored cell due to being the average location of the region
    || periodic_check_required
    ) {
#ifdef WALL_FOLLOWING_ENABLED
        //If we are not currently wall following
        if (wallFollower.wallFollowingDirection == 0) {
            //Find new frontier
            unexploredFrontierVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
        }
#else
        //Find new frontier
        unexploredFrontierVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
        this->last_feasibility_check_tick = this->elapsed_ticks;
#endif
    }

#else

#ifdef WALL_FOLLOWING_ENABLED
    if (this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT} || wallFollower.wallFollowingDirection == 0) {
        //Find new frontier
        unexploredFrontierVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
    }
#else
    //Find new frontier
        unexploredFrontierVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
#endif
#endif
#ifdef WALKING_STATE_WHEN_NO_FRONTIERS
    if (this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) {
        enterWalkingState(unexploredFrontierVector);
    } else {
        this->subTarget = {MAXFLOAT, MAXFLOAT};
    }
#endif

#ifdef AVOID_UNREACHABLE_FRONTIERS
    frontierEvaluator.updateConfidenceIfFrontierUnreachable(this);
#endif

    //If there are agents to avoid, do not explore
    if (agentAvoidanceVector.Length() != 0) unexploredFrontierVector = {0, 0};


    //Normalize vectors if they are not zero
    if (virtualWallAvoidanceVector.Length() != 0) virtualWallAvoidanceVector.Normalize();
    if (agentCohesionVector.Length() != 0) agentCohesionVector.Normalize();
    if (agentAvoidanceVector.Length() != 0) agentAvoidanceVector.Normalize();
    if (agentAlignmentVector.Length() != 0) agentAlignmentVector.Normalize();
    if (unexploredFrontierVector.Length() != 0) unexploredFrontierVector.Normalize();

    argos::CVector2 total_vector = calculateTotalVector(this->swarm_vector,
                                                        virtualWallAvoidanceVector,
                                                        agentCohesionVector,
                                                        agentAvoidanceVector,
                                                        agentAlignmentVector,
                                                        unexploredFrontierVector);
    argos::CRadians objectAvoidanceAngle;
    bool isThereAFreeAngle = ForceVectorCalculator::calculateObjectAvoidanceAngle(this, &objectAvoidanceAngle, total_vector.Angle());

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
    this->previousBestFrontier = this->currentBestFrontier;
    this->swarm_vector = total_vector;
}

#ifdef WALKING_STATE_WHEN_NO_FRONTIERS
/**
 * When there is no frontier to be found within the search range, select a random location on the edge of the root box.
 * @param unexploredFrontierVector
 */
void Agent::enterWalkingState(argos::CVector2 & unexploredFrontierVector) {
    argos::CVector2 agentToSubtarget = argos::CVector2(this->subTarget.x - this->position.x,
                                                       this->subTarget.y - this->position.y);;
    if (this->subTarget == Coordinate{MAXFLOAT, MAXFLOAT}
#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    || agentToSubtarget.Length() <= FRONTIER_DIST_UNTIL_REACHED
#endif
    ) {
        //Find a random direction to walk in, by placing a subtarget on the edge of the root box in the quadtree
        quadtree::Box rootBox = this->quadtree->getRootBox();
        Coordinate rootBoxCenter = rootBox.getCenter();
        double rootBoxSize = rootBox.getSize();
        argos::CRadians randomAngle = (rand() % 360) * argos::CRadians::PI /
                                      180; //TODO: Maybe away from average location of other agents?
        argos::CVector2 subtargetVector = argos::CVector2(1, 0);
        subtargetVector.Rotate(randomAngle);
        subtargetVector.Normalize();
        subtargetVector *= rootBoxSize * 0.5;
        this->subTarget = {rootBoxCenter.x + subtargetVector.GetX(), rootBoxCenter.y + subtargetVector.GetY()};
        agentToSubtarget = argos::CVector2(this->subTarget.x - this->position.x,
                                           this->subTarget.y - this->position.y);;

    }
    unexploredFrontierVector = agentToSubtarget;
}
#endif

argos::CVector2
Agent::calculateTotalVector(argos::CVector2 prev_total_vector, argos::CVector2 virtualWallAvoidanceVector,
                            argos::CVector2 agentCohesionVector,
                            argos::CVector2 agentAvoidanceVector,
                            argos::CVector2 agentAlignmentVector,
                            argos::CVector2 unexploredFrontierVector) {
    virtualWallAvoidanceVector = VIRTUAL_WALL_AVOIDANCE_WEIGHT * virtualWallAvoidanceVector;
    agentCohesionVector = AGENT_COHESION_WEIGHT * agentCohesionVector; //Normalize first
    agentAvoidanceVector = AGENT_AVOIDANCE_WEIGHT * agentAvoidanceVector;
    agentAlignmentVector = AGENT_ALIGNMENT_WEIGHT * agentAlignmentVector;
    unexploredFrontierVector = UNEXPLORED_FRONTIER_WEIGHT * unexploredFrontierVector;

    //According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    argos::CVector2 total_vector = prev_total_vector +
                                   virtualWallAvoidanceVector +
                                   agentCohesionVector +
                                   agentAvoidanceVector +
                                   agentAlignmentVector +
                                   unexploredFrontierVector;
    if (total_vector.Length() != 0) total_vector.Normalize();

    return total_vector;
}

void Agent::sendQuadtreeToCloseAgents() {
    std::vector<std::string> quadTreeToStrings = {};

    for (const auto& agentLocationPair: this->agentLocations) {
        double lastReceivedTick = std::get<2>(agentLocationPair.second);
        //If we have received the location of this agent in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds (so it is probably within communication range), send the quadtree
        if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second <
            AGENT_LOCATION_RELEVANT_DURATION_S) {
            //If we have not sent the quadtree to this agent yet in the past QUADTREE_EXCHANGE_INTERVAL_S seconds, send it
            if (!this->agentQuadtreeSent.count(agentLocationPair.first) ||
                this->elapsed_ticks - this->agentQuadtreeSent[agentLocationPair.first] > QUADTREE_EXCHANGE_INTERVAL_S * this->ticks_per_second) {
                if (quadTreeToStrings.empty()) { //Only calculate the quadtree if we have not done so yet
                    this->quadtree->toStringVector(&quadTreeToStrings);
                }
                for (const std::string &str: quadTreeToStrings) {
                    sendMessage("M:" + str, agentLocationPair.first);
                }
                this->agentQuadtreeSent[agentLocationPair.first] = this->elapsed_ticks; //Store the time we have sent the quadtree to this agent
            }
        }
    }

}

void Agent::doStep() {
    broadcastMessage("C:" + this->position.toString() + "|" + this->currentBestFrontier.toString());
    sendQuadtreeToCloseAgents();
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


        if (diffDeg > argos::CDegrees(-TURN_THRESHOLD_DEGREES) && diffDeg < argos::CDegrees(TURN_THRESHOLD_DEGREES)) {
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


    elapsed_ticks++;
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
void Agent::sendMessage(const std::string &message, const std::string& targetId) const {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    this->wifi.send_message(messagePrependedWithId, targetId);
}

/**
 * Check for messages from other agents
 * If there are messages, parse them
 */
void Agent::checkMessages() {
    //Read messages from other agents
    this->wifi.receive_messages(this->messages);
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
    std::string confidence;
    std::string visited;
    occPos = str.find(occDelimiter);
    coordinate = str.substr(0, occPos);
    quadtree::QuadNode newQuadNode{};
    newQuadNode.coordinate = coordinateFromString(coordinate);
    str.erase(0, occPos + occDelimiter.length());
    //Now we have the occupancy and ticks

    visitedPos = str.find(visitedDelimiter);
    confidence = str.substr(0, visitedPos);
    newQuadNode.LConfidence = static_cast<float>(std::stod(confidence));
    str.erase(0, visitedPos + visitedDelimiter.length());
    visited = str;
    newQuadNode.visitedAtS = std::stod(visited);
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
    std::map<std::string, int> agentQuadtreeBytesReceivedCounter;
    for (const std::string &message: this->messages) {
        std::string targetId = getTargetIdFromMessage(message);
        if (targetId != "A" && targetId != getId()) continue; //If the message is not broadcast to all agents and not for this agent, skip it
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent.at(0) == 'C') {
            auto splitStrings = splitString(messageContent.substr(2), "|");
            Coordinate receivedPosition = coordinateFromString(splitStrings[0]);
            Coordinate receivedFrontier = coordinateFromString(splitStrings[1]);
            this->agentLocations[senderId] = std::make_tuple(receivedPosition, receivedFrontier, this->elapsed_ticks);
        } else if (messageContent.at(0) == 'M') {
            std::vector<std::string> chunks;
            std::stringstream ss(messageContent.substr(2));
            std::string chunk;
            //Keep track of the total amount of bytes received from the sender
            if (agentQuadtreeBytesReceivedCounter.count(senderId) == 0){ //If the sender has not sent any bytes this tick yet
                agentQuadtreeBytesReceivedCounter[senderId] = messageContent.size();
            } else {
                agentQuadtreeBytesReceivedCounter[senderId] += messageContent.size();
            }
            while (std::getline(ss, chunk, '|')) {
                quadtree::QuadNode newQuadNode = quadNodeFromString(chunk);
                quadtree::Box addedBox = this->quadtree->add(newQuadNode, ALPHA_RECEIVE, elapsed_ticks / ticks_per_second);
#ifdef CLOSE_SMALL_AREAS
                if (newQuadNode.occupancy == quadtree::OCCUPIED && addedBox.getSize() != 0) // If the box is not the zero (not added)
                    checkIfAgentFitsBetweenObstacles(addedBox);
#endif

            }
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
    //Update the total amount of bytes received from each sender
    for (auto & it : agentQuadtreeBytesReceivedCounter){
        this->agentQuadtreeBytesReceived[it.first] = it.second;
    }

}

Radio Agent::getWifi() const {
    return this->wifi;
}

void Agent::setWifi(Radio newWifi) {
    this->wifi = newWifi;

}

std::vector<std::string> Agent::getMessages() {
    return this->messages;
}


