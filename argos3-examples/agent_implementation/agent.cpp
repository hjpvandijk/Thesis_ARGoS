//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include "agent.h"

#define PROXIMITY_RANGE 2.0

#define OBJECT_AVOIDANCE_WEIGHT 0.5
#define AGENT_AVOIDANCE_WEIGHT 0.5

//agent::agent() {
//    this->id = "";
//    this->position = new coordinate(0, 0);
//    this->speed = 0;
//}
//
//agent::agent(){
//    this->position = new coordinate(0, 0);
//    this->speed = 0;
//}

agent::agent(std::string id){
    this->id = id;
    this->position = {0.0, 0.0};
    this->heading = argos::CQuaternion();
    this->targetHeading = argos::CQuaternion();
    this->speed = 1;
    this->force_vector = argos::CVector2(0,0);
    this->messages = new std::vector<std::string>(0);
}

agent::agent(std::string id, coordinate position): id(id) {
    this->position = position;
}

void agent::setPosition(double new_x, double new_y) {
    argos::RLOG << "Setting position to: " << new_x << ", " << new_y << std::endl;
    this->position = {new_x, new_y};
}


void agent::setPosition(coordinate new_position) {
    this->position = new_position;
}

void agent::setHeading(argos::CQuaternion new_heading) {
    this->heading = new_heading;
};

void agent::setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator* diffdrive){
    this->diffdrive = diffdrive;
}


coordinate agent::getPosition() {
    return this->position;
}

std::string agent::getId() const {
    return this->id;
}

std::string agent::GetId() const {
    return this->id;
}

void agent::setId(std::string new_id) {
    this->id = new_id;
}

void agent::setSpeed(double new_speed) {
    this->speed = new_speed;
}

double agent::getSpeed() const {
    return this->speed;
}


void agent::print() {
    std::cout << "Agent " << this->id << " is at position (" << this->position.x << ", " << this->position.y
              << ")" << std::endl;
}

void agent::updateMap(){
    //Update map with new information
}

void agent::setLastRangeReading(double new_range) {
    this->lastRangeReading = new_range;
}

void agent::readDistanceSensor() {

}

void agent::readInfraredSensor() {

}

argos::CQuaternion agent::calculateObjectAvoidanceQT(){
    //If proximity < 0.1 create a new heading opposite of the current quaternion heading
    if(lastRangeReading < PROXIMITY_RANGE) {
        argos::RLOG << "Object avoidance" << std::endl;
        argos::RLOG << "Heading: " << heading << std::endl;
        argos::RLOG << "Inverse heading: " << heading.Inverse() << std::endl;
        return heading.Inverse();
    } else {
        return heading;
    }

}

argos::CQuaternion agent::calculateAgentAvoidanceQT() {
    for(auto agentLocation: agentLocations){
//        argos::RLOG << "Agent location: " << agentLocation.second.toString() << std::endl;
        //create a vector between this agent and the other agent
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(agentLocation.second.x, agentLocation.second.y)
                - argos::CVector2(position.x, position.y);
        //If the distance between the agents is less than a certain threshold, create a new heading opposite of the quaternion heading between the agent position and the other agent position
        if(vectorToOtherAgent.Length() < 2){
            argos::CRadians angle = vectorToOtherAgent.Angle();
//            argos::RLOG << "Angle between agents: " << ToDegrees(angle) << std::endl;
            argos::CQuaternion agentAvoidanceHeading;
            agentAvoidanceHeading.FromAngleAxis(angle, argos::CVector3(0,0,1));
            agentAvoidanceHeading = agentAvoidanceHeading.Inverse();
//            argos::RLOG << "Agent avoidance heading: " << agentAvoidanceHeading << std::endl;
//            argos::RLOG <<"Own heading: " << heading << std::endl;
            return agentAvoidanceHeading;
        } else {
            return heading;
        }
    }

    return heading;
}


argos::CQuaternion agent::averageQuaternions(std::vector<argos::CQuaternion> multipleRotations) {
    //Global variable which holds the amount of rotations which
//need to be averaged.
    int addAmount = 3;

//Global variable which represents the additive quaternion
    argos::CQuaternion addedRotation = argos::CQuaternion();

//The averaged rotational value
    argos::CQuaternion averageRotation;

//Loop through all the rotational values.
    for (argos::CQuaternion singleRotation: multipleRotations) {

        //Temporary values
        float w;
        float x;
        float y;
        float z;

        //Amount of separate rotational values so far
        addAmount++;

        float addDet = 1.0f / (float) addAmount;
        addedRotation.SetW(addedRotation.GetW() + singleRotation.GetW());
        w = addedRotation.GetW() * addDet;
        addedRotation.SetX(addedRotation.GetX() + singleRotation.GetX());
        w = addedRotation.GetX() * addDet;
        addedRotation.SetY(addedRotation.GetY() + singleRotation.GetY());
        w = addedRotation.GetY() * addDet;
        addedRotation.SetZ(addedRotation.GetZ() + singleRotation.GetZ());
        w = addedRotation.GetZ() * addDet;

        //Normalize. Note: experiment to see whether you
        //can skip this step.
        float D = 1.0f / (w * w + x * x + y * y + z * z);
        w *= D;
        x *= D;
        y *= D;
        z *= D;

        //The result is valid right away, without
        //first going through the entire array.
        averageRotation = argos::CQuaternion(x, y, z, w);
        return averageRotation;
    }
    return averageRotation;

}

void agent::calculateNextPosition() {
    argos::CQuaternion objectAvoidanceHeading=  calculateObjectAvoidanceQT();
    argos::CQuaternion agentAvoidanceHeading = calculateAgentAvoidanceQT();
    //Set target heading to the combination of the object avoidance and agent avoidance headings using their weights
    //Use the lerp technique to average the two quaternions

//    targetHeading = averageQuaternions({objectAvoidanceHeading, agentAvoidanceHeading});
    targetHeading = agentAvoidanceHeading;

}

void agent::doStep() {
    broadcastMessage("C:"+ position.toString());

    checkMessages();

    calculateNextPosition();

//    diffdrive->SetLinearVelocity(speed, speed);

    argos::RLOG << "Target heading: " << targetHeading << std::endl;
    argos::RLOG << "Heading: " << heading << std::endl;

    argos::CQuaternion diff = heading * targetHeading.Inverse();

    argos::CRadians cZAngle, cYAngle, cXAngle;
    diff.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    argos::CDegrees cZAngleDeg = ToDegrees(cZAngle);
    argos::RLOG << "Angle between headings: " << cZAngleDeg << std::endl;

    if (cZAngleDeg > argos::CDegrees(-5) && cZAngleDeg < argos::CDegrees(5)){
        //Go straight
        diffdrive->SetLinearVelocity(speed, speed);
        argos::RLOG << "Going straight" << std::endl;
    }
    else if(cZAngleDeg > argos::CDegrees(0)){
        //turn right
        diffdrive->SetLinearVelocity(speed, 0);
        argos::RLOG << "Turning right" << std::endl;

    } else {
        //turn left
        diffdrive->SetLinearVelocity(0, speed);
        argos::RLOG << "Turning left" << std::endl;

    }


//    argos::CRadians angle;
//    argos::CVector3 axis;
//    diff.ToAngleAxis(angle, axis);
//    argos::RLOG << "Angle between headings: " << ToDegrees(angle) << std::endl;
//    argos::RLOG << "Axis between headings: " << axis << std::endl;
//    if (ToDegrees(angle) < argos::CDegrees(1) || ToDegrees(angle) > argos::CDegrees(359)){
//        //Go straight
//        diffdrive->SetLinearVelocity(speed, speed);
//        argos::RLOG << "Going straight" << std::endl;
//    }
//    else if(ToDegrees(angle)> argos::CDegrees(180)){
//        //turn right
//        diffdrive->SetLinearVelocity(speed, 0);
//        argos::RLOG << "Turning right" << std::endl;
//
//    } else {
//        //turn left
//        diffdrive->SetLinearVelocity(0, speed);
//        argos::RLOG << "Turning left" << std::endl;
//
//    }
}

//TODO: send bytes instead of string
void agent::broadcastMessage(std::string message) {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    argos::UInt8* buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff,messagePrependedWithId.size()+1);
    wifi.broadcast_message(cMessage);
//    argos::LOG << "[" << getId() << "] " << "Broadcasted message: " << message << std::endl;
}

void agent::checkMessages() {
    //Read messages from other agents
    wifi.receive_messages(messages);
//    for (const auto & message : *messages) {
//        argos::LOG << "[" << getId() << "] " << "Received message: " << message << std::endl;
//    }
    if(!messages->empty()) parseMessages();

}

std::string getIdFromMessage(std::string message){
    return message.substr(1, message.find(']')-1);

}

coordinate coordinateFromString(std::string str) {
    std::string delimiter = ";";
    size_t pos = 0;
    std::string token;
    pos = str.find(delimiter);
    token = str.substr(0, pos);
    coordinate newCoordinate;
    newCoordinate.x = std::stod(token);
    str.erase(0, pos + delimiter.length());
    newCoordinate.y = std::stod(str);
    return newCoordinate;
}

void agent::parseMessages() {
    for(std::string message: *messages){
        std::string senderId = getIdFromMessage(message);
        std::string messageContent = message.substr(message.find(']')+1);
        if(messageContent[0] == 'C'){
            coordinate receivedPosition = coordinateFromString(messageContent.substr(2));
            agentLocations[senderId] = receivedPosition;
        }
    }

}

radio agent::getWifi() const {
    return this->wifi;
}

void agent::setWifi(radio wifi) {
    this->wifi = wifi;

}

std::vector<std::string> agent::getMessages() {
    return *this->messages;
}


//void calculateNextPosition() {
//
//    //Inspired by boids algorithm:
//        //Vector determining heading
//        //Vector is composed of:
//            //1. Attraction to unexplored frontier
//            //2. Repulsion from other agents
//            //3. Attraction to found target
//            //4. Repulsion from objects/walls
//
//}