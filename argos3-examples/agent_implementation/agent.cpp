//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include "agent.h"



Agent::Agent(std::string id) {
    this->id = id;
    this->position = {0.0, 0.0};
    this->heading = argos::CRadians(0);
    this->targetHeading = argos::CRadians(0);
    this->speed = 5;
    this->force_vector = argos::CVector2(0, 1);
    this->messages = new std::vector<std::string>(0);
    auto box = quadtree::Box(-5,5,10);
    this->quadtree = new quadtree::Quadtree(box);
}

int i = 0;

Agent::Agent(std::string id, Coordinate new_position) : id(id) {
    this->position = new_position;
}

void Agent::setPosition(double new_x, double new_y) {
    this->position = {new_x, new_y};
    argos::RLOG << " is at position (" << this->position.x << ", " << this->position.y
                << ")" << std::endl;
    quadtree->add(Coordinate{this->position.x, this->position.y}, quadtree::Occupancy::FREE);
    i++;
    if(i%100==0) quadtree->exportQuadtreeToFile(this->getId());
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

void Agent::setLastRangeReading(double new_range) {
    this->lastRangeReading = new_range;
}

void Agent::readDistanceSensor() {

}

void Agent::readInfraredSensor() {

}

void Agent::addFreeAreaBetween(Coordinate agentCoordinate, Coordinate objectCoordinate){
    double x = agentCoordinate.x;
    double y = agentCoordinate.y;
    double dx = objectCoordinate.x - agentCoordinate.x;
    double dy = objectCoordinate.y - agentCoordinate.y;
    double distance = sqrt(dx*dx + dy*dy);
    double stepSize = quadtree->getMinSize();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for(int s = 0; s < nSteps; s++){
        quadtree->add(Coordinate{x, y}, quadtree::Occupancy::FREE);
        x += stepX;
        y += stepY;
    }
}

void Agent::addObjectLocation(Coordinate agentCoordinate, Coordinate objectCoordinate){
    quadtree->add(objectCoordinate, quadtree::Occupancy::OCCUPIED);
}

void Agent::checkForObstacles() {
    if (this->lastRangeReading < PROXIMITY_RANGE) {

        argos::RLOG << "Heading: " << this->heading << std::endl;
        argos::RLOG << "Last range reading: " << this->lastRangeReading << std::endl;
        double opposite = argos::Sin(this->heading) * this->lastRangeReading;
        double adjacent = argos::Cos(this->heading) * this->lastRangeReading;

        argos::RLOG << "Opposite: " << opposite << std::endl;
        argos::RLOG << "Adjacent: " << adjacent << std::endl;

        Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
        addFreeAreaBetween(this->position, object);
        addObjectLocation(this->position, object);
    } else {
        double opposite = argos::Sin(this->heading) * PROXIMITY_RANGE;
        double adjacent = argos::Cos(this->heading) * PROXIMITY_RANGE;

        argos::RLOG << "Opposite: " << opposite << std::endl;
        argos::RLOG << "Adjacent: " << adjacent << std::endl;

        Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
        addFreeAreaBetween(this->position, end_of_ray);
    }
}

/**
 * Calculate the vector to avoid objects:
 * If the rangefinder reading is less than the set PROXIMITY RANGE,
 * meaning there is an object in front of the agent,
 * create a vector in the opposite direction of the object.
 * @return a vector pointing away from the object
 */
argos::CVector2 Agent::calculateObjectAvoidanceVector() {

        std::vector<quadtree::QuadNode> occupiedNodes = quadtree->queryOccupied(this->position, PROXIMITY_RANGE*2.0);

        argos::RLOG << "Occupied nodes: " << occupiedNodes.size() << std::endl;

        argos::CVector2 totalObjectRepulsionVector = {0, 0};

        //Calculate a repulsion vector for each object within range
        for(auto node: occupiedNodes){
            Coordinate nodeCoordinate = node.coordinate;
            argos::CVector2 vectorToObject =
                    argos::CVector2(nodeCoordinate.x, nodeCoordinate.y)
                    - argos::CVector2(this->position.x, this->position.y);

            //Reverse the vector
            totalObjectRepulsionVector += (vectorToObject * -1);
        }

        //Normalize the vector
        if (totalObjectRepulsionVector.Length() != 0) totalObjectRepulsionVector.Normalize();

        return totalObjectRepulsionVector;


}

/**
 * Calculate the vector to avoid other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the opposite direction of the average location of these agents.
 * @return a vector pointing away from the average location other agents
 */
argos::CVector2 Agent::calculateAgentAvoidanceVector() {
    int nAgentsWithinRange = 0;
    Coordinate averageNeighborLocation = {0, 0};
    for (auto agentLocation: this->agentLocations) {
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(agentLocation.second.x, agentLocation.second.y)
                - argos::CVector2(this->position.x, this->position.y);

        if (vectorToOtherAgent.Length() < AGENT_AVOIDANCE_RANGE) {
            averageNeighborLocation.x += agentLocation.second.x;
            averageNeighborLocation.y += agentLocation.second.y;
            nAgentsWithinRange++;
        }
    }
    //If no agents are within range, return a zero vector
    if (nAgentsWithinRange == 0) return {0, 0};

    //Else calculate the average position of the agents within range
    averageNeighborLocation.x /= nAgentsWithinRange;
    averageNeighborLocation.y /= nAgentsWithinRange;

    //Create a vector between this agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgent =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(this->position.x, this->position.y);

    //Reverse the vector
    vectorToOtherAgent = vectorToOtherAgent * -1;

    return vectorToOtherAgent;
}

void Agent::calculateNextPosition() {
    //Inspired by boids algorithm:
        //Vector determining heading
        //Vector is composed of:
            //1. Attraction to unexplored frontier
            //2. Repulsion from other agents
            //3. Attraction to found target
            //4. Repulsion from objects/walls

    argos::CVector2 objectAvoidanceVector = calculateObjectAvoidanceVector();
    argos::CVector2 agentAvoidanceVector = calculateAgentAvoidanceVector();

    argos::RLOG << "Object avoidance vector: " << objectAvoidanceVector << std::endl;
    argos::RLOG << "Agent avoidance vector: " << agentAvoidanceVector << std::endl;

    //Normalize vectors if they are not zero
    if (objectAvoidanceVector.Length() != 0) objectAvoidanceVector.Normalize();
    if (agentAvoidanceVector.Length() != 0) agentAvoidanceVector.Normalize();

    argos::CVector2 total_vector = this->force_vector + OBJECT_AVOIDANCE_WEIGHT * objectAvoidanceVector +
                                   AGENT_AVOIDANCE_WEIGHT * agentAvoidanceVector;
    if (total_vector.Length() != 0) total_vector.Normalize();

    this->force_vector = total_vector;

    argos::RLOG << "Total vector: " << total_vector << std::endl;

    argos::CRadians angle = total_vector.Angle();
    this->targetHeading = angle;
}

void Agent::doStep() {
    broadcastMessage("C:" + this->position.toString());

    checkMessages();

    checkForObstacles();

    calculateNextPosition();

    argos::RLOG << "Heading: " << ToDegrees(this->heading) << std::endl;
    argos::RLOG << "Target heading: " << ToDegrees(this->targetHeading) << std::endl;

    argos::CRadians diff = (this->heading - this->targetHeading).SignedNormalize();

    argos::CDegrees diffDeg = ToDegrees(diff);

    argos::RLOG << "Diff: " << diffDeg << std::endl;

    if (diffDeg > argos::CDegrees(-10) && diffDeg < argos::CDegrees(10)) {
        //Go straight
        this->diffdrive->SetLinearVelocity(this->speed, this->speed);
//        argos::RLOG << "Going straight" << std::endl;
    } else if (diffDeg > argos::CDegrees(0)) {
        //turn right
        this->diffdrive->SetLinearVelocity(this->speed*TURNING_SPEED_RATIO, 0);
//        argos::RLOG << "Turning right" << std::endl;

    } else {
        //turn left
        this->diffdrive->SetLinearVelocity(0, this->speed*TURNING_SPEED_RATIO);
//        argos::RLOG << "Turning left" << std::endl;

    }

    argos::RLOG << std::endl;

}

void Agent::broadcastMessage(std::string message) {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    argos::UInt8 *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    this->wifi.broadcast_message(cMessage);
}

void Agent::checkMessages() {
    //Read messages from other agents
    this->wifi.receive_messages(this->messages);
    if (!this->messages->empty()) parseMessages();

}

std::string getIdFromMessage(std::string message) {
    return message.substr(1, message.find(']') - 1);

}

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

void Agent::parseMessages() {
    for (std::string message: *this->messages) {
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent[0] == 'C') {
            Coordinate receivedPosition = coordinateFromString(messageContent.substr(2));
            this->agentLocations[senderId] = receivedPosition;
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
