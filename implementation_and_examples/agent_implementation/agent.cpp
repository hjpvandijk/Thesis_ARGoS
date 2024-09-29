//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include <set>
#include "agent.h"
#include <random>
#include <utility>


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

void Agent::setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *newDiffdrive) {
    this->diffdrive = newDiffdrive;
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
    this->lastRangeReadings.at(index) = new_range;
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
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2) const {
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
        this->quadtree->add(Coordinate{x, y}, quadtree::Occupancy::FREE, elapsed_ticks / ticks_per_second);
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

    for (int s = 0; s < nSteps; s++) {
        this->quadtree->add(Coordinate{x, y}, quadtree::Occupancy::OCCUPIED, elapsed_ticks / ticks_per_second);
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


    for (int s = 0; s < nSteps; s++) {
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
void Agent::addObjectLocation(Coordinate objectCoordinate) const {
    quadtree::Box objectBox = this->quadtree->add(objectCoordinate, quadtree::Occupancy::OCCUPIED,
                                                  elapsed_ticks / ticks_per_second);
    if(this->CLOSE_SMALL_AREAS) checkIfAgentFitsBetweenObstacles(objectBox);

}

/**
 * Check for obstacles in front of the agent
 * If there is an obstacle within a certain range, add the free area between the agent and the obstacle to the quadtree
 * If there is no obstacle within range, add the free area between the agent and the end of the range to the quadtree
 */
void Agent::checkForObstacles() {
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        if (this->lastRangeReadings[sensor_index] < PROXIMITY_RANGE) {

            double opposite = argos::Sin(sensor_rotation) * this->lastRangeReadings[sensor_index];
            double adjacent = argos::Cos(sensor_rotation) * this->lastRangeReadings[sensor_index];


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
                if (objectToAgent.Length() <= this->quadtree->getSmallestBoxSize()) {
                    close_to_other_agent = true;
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if (!close_to_other_agent) addObjectLocation(object);


        } else {
            double opposite = argos::Sin(sensor_rotation) * PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * PROXIMITY_RANGE;


            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
            addFreeAreaBetween(this->position, end_of_ray);
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
                                                                                  4 * AGENT_SAFETY_RADIUS,
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


// Custom comparator to order set for wall following. The set is ordered by the angle difference to the wall following direction
struct CustomComparator {
    int dir;  // dir is either 0, 1 or -1
    double headingRounded;

    CustomComparator(int dir, double headingRounded) : dir(dir), headingRounded(headingRounded) {}


    //SOMETHING GOES WRONG WITH ANGLE 122 AND HEADING 32 --> diff = 90 exactly
    //Good with heading 36 --> 86
    // Custom comparator logic
    bool operator()(const argos::CDegrees &a, const argos::CDegrees &b) const {


        auto a_diff = a - argos::CDegrees(this->headingRounded);
        auto b_diff = b - argos::CDegrees(this->headingRounded);

        a_diff.SignedNormalize();
        b_diff.SignedNormalize();

        auto a_diff_val = a_diff.GetValue();
        auto b_diff_val = b_diff.GetValue();

        if (dir >= 0) {
            // Handle the first half: 90 to -180
            if (a_diff_val <= 90 && a_diff_val >= -180 && b_diff_val <= 90 && b_diff_val >= -180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Handle the second half: 180 to 91
            if (a_diff_val > 90 && a_diff_val <= 180 && b_diff_val > 90 && b_diff_val <= 180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (90 to -180) over the second half (180 to 91)
            if ((a_diff_val <= 90 && a_diff_val >= -180) && (b_diff_val > 90 && b_diff_val <= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val > 90 && a_diff_val <= 180) && (b_diff_val <= 90 && b_diff_val >= -180)) {
                return false;  // 'b' should come before 'a'
            }
        } else {
            // Handle the first half: -90 to 180
            if (a_diff_val >= -90 && a_diff_val <= 180 && b_diff_val >= -90 && b_diff_val <= 180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Handle the second half: -180 to -91
            if (a_diff_val < -90 && a_diff_val >= -180 && b_diff_val < -90 && b_diff_val >= -180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (-90 to 180) over the second half (-180 to -91)
            if ((a_diff_val >= -90 && a_diff_val <= 180) && (b_diff_val < -90 && b_diff_val >= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val < -90 && a_diff_val >= -180) && (b_diff_val >= -90 && b_diff_val <= 180)) {
                return false;  // 'b' should come before 'a'
            }
        }

        return a_diff_val > b_diff_val;  // Default to descending order if somehow unmatched
    }
};

/**
 * Calculate the vector to avoid objects:
 * Find the closest relative angle that is free of objects within a certain range.
 * According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * @return The vector towards the free direction
 */
bool Agent::calculateObjectAvoidanceAngle(argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {

    //Get occupied boxes within range
    std::vector<quadtree::Box> occupiedBoxes = this->quadtree->queryOccupiedBoxes(this->position,
                                                                                  OBJECT_AVOIDANCE_RADIUS * 2,
                                                                                  this->elapsed_ticks /
                                                                                  this->ticks_per_second);

    double angleInterval = argos::CDegrees(360 / ANGLE_INTERVAL_STEPS).GetValue();
    auto headingRounded = (int) (ToDegrees(this->heading).GetValue() / angleInterval) * angleInterval;

    //Create set of free angles ordered to be used for wall following
    std::set<argos::CDegrees, CustomComparator> freeAngles(
            CustomComparator(this->wallFollowingDirection, headingRounded));

//Add free angles from -180 to 180 degrees
    for (int a = 0; a < ANGLE_INTERVAL_STEPS; a++) {
        auto angle = argos::CDegrees(a * 360 / ANGLE_INTERVAL_STEPS - 180);
        freeAngles.insert(angle);
    }



    //For each occupied box, find the angles that are blocked relative to the agent
    for (auto box: occupiedBoxes) {


        argos::CVector2 OC = argos::CVector2(box.getCenter().x - this->position.x,
                                             box.getCenter().y - this->position.y);
        argos::CRadians Bq = argos::ASin(
                std::min(AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
        argos::CRadians Eta_q = OC.Angle();
        if (AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS > OC.Length())
            argos::RLOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << AGENT_SAFETY_RADIUS
                           << " + " << OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;

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
        for (int a = 0; a < ANGLE_INTERVAL_STEPS; a++) {
            auto angle = argos::CDegrees(a * 360 / ANGLE_INTERVAL_STEPS - 180);

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

    if (this->WALL_FOLLOWING_ENABLED) { //If wall following is enabled
        if (this->previousBestFrontier == this->currentBestFrontier) { // If we are still on route to the same frontier
            if (std::abs(ToDegrees(*relativeObjectAvoidanceAngle).GetValue()) >
                89) { //The complete forward direction to the target is blocked
                argos::CVector2 agentToHitPoint = argos::CVector2(this->wallFollowingHitPoint.x - this->position.x,
                                                                  this->wallFollowingHitPoint.y - this->position.y);
                if (agentToHitPoint.Length() <= this->quadtree->getSmallestBoxSize()) {

                    if (this->wallFollowingDirection == 0) {
                        this->wallFollowingDirection = 1;
                    } else if (this->wallFollowingDirection == 1) {
                        this->wallFollowingDirection = -1;
                    } else {
                        if (!this->lastIterationInHitPoint) {
                            this->wallFollowingDirection = 1;
                        }
                    }
                    this->lastIterationInHitPoint = true;

                } else {
                    this->wallFollowingDirection = 1; //TODO: Make random?
                }
                this->wallFollowingHitPoint = this->position;
            } else if (std::abs(ToDegrees(*relativeObjectAvoidanceAngle).GetValue()) <
                       10) { //Direction to the frontier is free again.

                this->wallFollowingDirection = 0;
            }
            argos::CVector2 agentToHitPoint = argos::CVector2(this->wallFollowingHitPoint.x - this->position.x,
                                                              this->wallFollowingHitPoint.y - this->position.y);
            if (agentToHitPoint.Length() > this->quadtree->getSmallestBoxSize()) {
                this->lastIterationInHitPoint = false;
            }
        } else {
            this->wallFollowingDirection = 0;
            this->lastIterationInHitPoint = false;
        }



        //Wall following:
        //Choose free direction closest to current heading
        //If no wall on the wall following side of the robot, turn that way
        //Loop until direction to frontier is free.
        if (wallFollowingDirection != 0) { // If we are in wall following mode
            //Get the closest free angle to the wall following direction (90 degrees right or left)
            //Create a subtarget in that direction
            argos::CDegrees subtargetAngle = *freeAngles.begin();
            argos::CVector2 subtargetVector = argos::CVector2(1, 0);
            subtargetVector.Rotate(ToRadians(subtargetAngle));
            subtargetVector.Normalize();
            subtargetVector *= this->OBJECT_AVOIDANCE_RADIUS;
            this->subTarget = {this->position.x + subtargetVector.GetX(), this->position.y + subtargetVector.GetY()};

            closestFreeAngle = subtargetAngle;
            closestFreeAngleRadians = ToRadians(closestFreeAngle);
            *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);
        }
    }

    return true;

}

/** Calculate the vector to avoid the virtual walls
 * If the agent is close to the border, create a vector pointing away from the border
 * Implemented according to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * However, the vector directions are flipped compared to the paper, as the paper uses a different coordinate system
 * @return a vector pointing away from the border
 *
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
                argos::CVector2(agentLocation.second.x, agentLocation.second.y)
                - argos::CVector2(this->position.x, this->position.y);

        if (vectorToOtherAgent.Length() < range) { //TODO: make different for cohesion and separation
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

    bool neighborsWithinRange = getAverageNeighborLocation(&averageNeighborLocation, AGENT_COHESION_RADIUS);
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

    bool neighborsWithinRange = getAverageNeighborLocation(&averageNeighborLocation, AGENT_AVOIDANCE_RADIUS);
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
        Coordinate otherAgentLocation = agentLocations[agentID];
        argos::CVector2 agentVector = agentVelocity.second.first;
        double agentSpeed = agentVelocity.second.second;
        argos::CVector2 vectorToOtherAgent = argos::CVector2(otherAgentLocation.x, otherAgentLocation.y)
                                             - argos::CVector2(this->position.x, this->position.y);
        if (vectorToOtherAgent.Length() < AGENT_ALIGNMENT_RADIUS) {
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
    current_frontiers = frontiers;

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

    current_frontier_regions = frontierRegions;

    //Now we have all frontier cells merged into frontier regions
    //Find F* by using the formula above
    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
    //Ψ_S = FRONTIER_SIZE_WEIGHT

    //Initialize variables to store the best frontier region and its score
    std::vector<quadtree::Box> bestFrontierRegion = {};
    Coordinate bestFrontierRegionCenter = {0, 0};
    double bestFrontierScore = std::numeric_limits<double>::max();

    //Iterate over all frontier regions to find the best one
    for (const auto &region: frontierRegions) {
        //Calculate the average position of the frontier region
        double sumX = 0;
        double sumY = 0;
        double totalNumberOfCellsInRegion = 0;
        for (auto box: region) {
            double cellsInBox = box.getSize() / quadtree->getSmallestBoxSize();
            assert(cellsInBox == 1);
            sumX += box.getCenter().x *
                    cellsInBox; //Take the box size into account (parent nodes will contain the info about all its children)
            sumY += box.getCenter().y * cellsInBox;
            totalNumberOfCellsInRegion += cellsInBox;
        }
        double frontierRegionX = sumX / totalNumberOfCellsInRegion;
        double frontierRegionY = sumY / totalNumberOfCellsInRegion;

        if (this->BLACKLIST_FRONTIERS) {
            argos::RLOG << "Checking if frontier: " << frontierRegionX << ", " << frontierRegionY
                        << " is blacklisted" << std::endl;
            bool blacklisted = false;
            for (auto &blacklistedFrontier: this->blacklistedFrontiers) {
                Coordinate blackListedFrontierCoordinate = blacklistedFrontier.first;
                argos::RLOG << "Calculating distance between :" << frontierRegionX << ", " << frontierRegionY
                            << " and " << blackListedFrontierCoordinate.x << ", "
                            << blackListedFrontierCoordinate.y << std::endl;
                double distanceBetweenFrontiers = sqrt(pow(frontierRegionX - blackListedFrontierCoordinate.x, 2) +
                                                       pow(frontierRegionY - blackListedFrontierCoordinate.y, 2));
                argos::RLOG << "Distance between frontiers: " << distanceBetweenFrontiers << std::endl;
                argos::RLOG << "Min allowed distance between frontiers: " << this->minAllowedDistanceBetweenFrontiers
                            << std::endl;
                if (distanceBetweenFrontiers < this->minAllowedDistanceBetweenFrontiers) {
                    int timesBlacklisted = blacklistedFrontier.second.first;
                    double blacklistChance = 100-(timesBlacklisted * this->blacklistChancePerCount * 100);
                    double randomChance = rand() % 100;
                    argos::RLOG << "Blacklist chance: " << blacklistChance << std::endl;
                    argos::RLOG << "avoiding? : " << blacklistedFrontier.second.second << " || " << randomChance << " > " << blacklistChance << std::endl;
                    if (blacklistedFrontier.second.second || randomChance > blacklistChance) {//If currently already avoiding, or not current target and random chance to blacklist the frontier
                        argos::RLOG << "Blacklisted frontier so skip" << std::endl;
                        blacklisted = true;
                        blacklistedFrontier.second.second = true;
                    }
                    break;
                }
            }
            if (blacklisted) continue; //Skip this frontier
        }

        //Calculate the distance between the agent and the frontier region
        double distance = sqrt(pow(frontierRegionX - this->position.x, 2) + pow(frontierRegionY - this->position.y, 2));

        //Calculate the score of the frontier region
        double score = FRONTIER_DISTANCE_WEIGHT * distance - FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion;

        if (this->SEPARATE_FRONTIERS) {
            std::vector<double> distancesFromOtherAgents = {};

            for (const auto &agentLocationPair: this->agentLocations) {
                //Get the distance between the frontier and the last known location of other agents
                Coordinate agentLocation = agentLocationPair.second;
                double distanceFromOtherAgent = sqrt(
                        pow(frontierRegionX - agentLocation.x, 2) + pow(frontierRegionY - agentLocation.y, 2));
                distancesFromOtherAgents.push_back(distanceFromOtherAgent);
            }

            //If that frontier is best to visit for a different agent, skip it.
            bool otherAgentLowerScore = false;
            for (auto distanceFromOtherAgent: distancesFromOtherAgents) {
                double otherAgentScore = FRONTIER_DISTANCE_WEIGHT * distanceFromOtherAgent -
                                         FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion;
                if (otherAgentScore < score) {
                    otherAgentLowerScore = true;
                    break;
                }
            }
            if (otherAgentLowerScore) continue;
        }

        //If the score is lower than the best score, update the best score and best frontier region
        if (score < bestFrontierScore) {
            bestFrontierScore = score;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
        }
    }

    this->currentBestFrontier = bestFrontierRegionCenter;



    //Calculate the vector to the best frontier region
    argos::CVector2 vectorToBestFrontier = argos::CVector2(bestFrontierRegionCenter.x, bestFrontierRegionCenter.y)
                                           - argos::CVector2(this->position.x, this->position.y);

    return vectorToBestFrontier;
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
    argos::CVector2 agentAvoidanceVector = calculateAgentAvoidanceVector();
    argos::CVector2 agentAlignmentVector = calculateAgentAlignmentVector();

    argos::CVector2 unexploredFrontierVector = argos::CVector2(this->currentBestFrontier.x - this->position.x,
                                                               this->currentBestFrontier.y - this->position.y);

    if (this->BLACKLIST_FRONTIERS) {
        //If the agent is close to the frontier, reset all blacklisted frontiers avoiding flags
        if (unexploredFrontierVector.Length() <= this->quadtree->getSmallestBoxSize()) {
            //Reset all 'avoiding' flags
            for (auto &blacklistedFrontier: this->blacklistedFrontiers) {
                blacklistedFrontier.second.second = false;
            }
        }
    }

    if(this->DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED) {
        argos::RLOG << "blacklisted? " << (this->blacklistedFrontiers.find(this->currentBestFrontier) != this->blacklistedFrontiers.end()) << std::endl;
        argos::RLOG << "current best frontier: " << this->currentBestFrontier.x << ", " << this->currentBestFrontier.y << std::endl;
        for (auto &blacklistedFrontier: this->blacklistedFrontiers) {
            argos::RLOG << "blacklisted frontier: " << blacklistedFrontier.first.x << ", " << blacklistedFrontier.first.y << std::endl;
            argos::RLOG << (this->currentBestFrontier == blacklistedFrontier.first) << std::endl;
        }
        if (this->blacklistedFrontiers.find(this->currentBestFrontier) != this->blacklistedFrontiers.end() || //If the current best frontier is blacklisted
            unexploredFrontierVector.Length() <= this->quadtree->getSmallestBoxSize()) { //Or the agent is close to the frontier
            //Find new frontier
            unexploredFrontierVector = calculateUnexploredFrontierVector();
        }

    } else if (wallFollowingDirection == 0) {
        unexploredFrontierVector = calculateUnexploredFrontierVector();
    }


    if (this->BLACKLIST_FRONTIERS) {
        double distanceToBestFrontier = sqrt(pow(this->currentBestFrontier.x - this->position.x, 2) +
                                             pow(this->currentBestFrontier.y - this->position.y, 2));
        if (this->currentBestFrontier == this->previousBestFrontier) { //If we are still on route to the same frontier
            //Check if the distance to the frontier has decreased in the last timeToCheckFrontierDistS seconds
            argos::RLOG << "Distance to best frontier: " << distanceToBestFrontier << std::endl;
            argos::RLOG << "Min distance to best frontier: " << this->minDistFromFrontier << std::endl;
            if (distanceToBestFrontier < this->minDistFromFrontier) { //If the distance has decreased
                argos::RLOG << "Distance to best frontier decreased" << std::endl;
                this->minDistFromFrontier = distanceToBestFrontier;
                this->timeFrontierDistDecreased = this->elapsed_ticks / this->ticks_per_second;
            } else if (this->elapsed_ticks / this->ticks_per_second - this->timeFrontierDistDecreased >
                       timeToCheckFrontierDistS) {
                //If the distance has not decreased in the last timeToCheckFrontierDistS seconds, blacklist the frontier
                argos::RLOG << "Distance to best frontier did not decrease in the last " << timeToCheckFrontierDistS
                            << " seconds" << std::endl;
                bool sameAsOtherFrontier = false;
                for (auto &blacklistedFrontier: this->blacklistedFrontiers) {
                    Coordinate blackListedFrontierCoordinate = blacklistedFrontier.first;
                    double distanceBetweenFrontiers = sqrt(
                            pow(this->currentBestFrontier.x - blackListedFrontierCoordinate.x, 2) +
                            pow(blackListedFrontierCoordinate.y - this->position.y, 2));
                    if (distanceBetweenFrontiers < this->minAllowedDistanceBetweenFrontiers) {
                        blacklistedFrontier.second.first++;
                        sameAsOtherFrontier = true;
                    }
                }
                if (!sameAsOtherFrontier) {
                    argos::RLOG << "Blacklisting frontier: " << this->currentBestFrontier.x << ", "
                                << this->currentBestFrontier.y << std::endl;
                    this->blacklistedFrontiers[this->currentBestFrontier] = std::make_pair<int, bool>(1, false);
                }
            }

        } else { //If we are not on route to the same frontier, set the min distance and time
            this->minDistFromFrontier = distanceToBestFrontier;
            this->timeFrontierDistDecreased = this->elapsed_ticks / this->ticks_per_second;
        }
    }

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
    bool isThereAFreeAngle = calculateObjectAvoidanceAngle(&objectAvoidanceAngle, total_vector.Angle());

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

void Agent::doStep() {
    broadcastMessage("C:" + this->position.toString());
    std::vector<std::string> quadTreeToStrings = {};
    this->quadtree->toStringVector(&quadTreeToStrings);
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
    if (this->force_vector == argos::CVector2{0, 0}) this->diffdrive->SetLinearVelocity(0, 0);
    else {

        argos::CRadians diff = (this->heading - this->targetHeading).SignedNormalize();

        argos::CDegrees diffDeg = ToDegrees(diff);


        if (diffDeg > argos::CDegrees(-TURN_THRESHOLD_DEGREES) && diffDeg < argos::CDegrees(TURN_THRESHOLD_DEGREES)) {
            //Go straight
            this->diffdrive->SetLinearVelocity(this->speed, this->speed);
        } else if (diffDeg > argos::CDegrees(0)) {
            //turn right
            this->diffdrive->SetLinearVelocity(this->speed * TURNING_SPEED_RATIO, 0);

        } else {
            //turn left
            this->diffdrive->SetLinearVelocity(0, this->speed * TURNING_SPEED_RATIO);

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
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
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
    if (!this->messages.empty()) parseMessages();

}

/**
 * Get the id from a message
 * @param message
 * @return
 */
std::string getIdFromMessage(const std::string &message) {
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
    std::string occ;
    std::string visited;
    occPos = str.find(occDelimiter);
    coordinate = str.substr(0, occPos);
    quadtree::QuadNode newQuadNode{};
    newQuadNode.coordinate = coordinateFromString(coordinate);
    str.erase(0, occPos + occDelimiter.length());
    //Now we have the occupancy and ticks

    visitedPos = str.find(visitedDelimiter);
    occ = str.substr(0, visitedPos);
    newQuadNode.occupancy = static_cast<quadtree::Occupancy>(std::stoi(occ));
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
    for (const std::string &message: this->messages) {
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent.at(0) == 'C') {
            Coordinate receivedPosition = coordinateFromString(messageContent.substr(2));
            this->agentLocations[senderId] = receivedPosition;
        } else if (messageContent.at(0) == 'M') {
            std::vector<std::string> chunks;
            std::stringstream ss(messageContent.substr(2));
            std::string chunk;

            while (std::getline(ss, chunk, '|')) {
                this->quadtree->add(quadNodeFromString(chunk));
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
