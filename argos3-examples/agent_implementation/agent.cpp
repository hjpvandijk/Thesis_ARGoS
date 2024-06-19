//
// Created by hugo on 17-6-24.
//

#include <iostream>
#include <argos3/core/utility/logging/argos_log.h>
#include "agent.h"

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
    this->position = new coordinate(0.0,0.0);
    this->speed = 20;
    this->messages = new std::vector<std::string>(0);
}

agent::agent(std::string id, coordinate *position): id(id) {
    this->position = position;
}

void agent::setPosition(double new_x, double new_y) const {
    this->position->setCoordinates(new_x, new_y);
}


void agent::setPosition(coordinate *new_position) {
    this->position = new_position;
}

coordinate *agent::getPosition() const {
    return this->position;
}

std::string agent::getId() const {
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
    std::cout << "Agent " << this->id << " is at position (" << this->position->x << ", " << this->position->y
              << ")" << std::endl;
}

void agent::updateMap(){
    //Update map with new information
}

void agent::readDistanceSensor() {

}

void agent::readInfraredSensor() {

}

void agent::calculateNextPosition() {

}

void agent::broadcastMessage(std::string message) {
    argos::UInt8* buff = (argos::UInt8 *) message.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff,message.size()+1);
    wifi.broadcast_message(cMessage);
}

void agent::readMessages() {
    //Read messages from other agents
    wifi.receive_messages(messages);
    for (int i = 0; i < messages->size(); i++) {
        argos::LOG << "[" << getId() << "] " << "Sensor message agent: " << (*messages)[i] << std::endl;
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


void calculateNextPosition() {
    //Inspired by boids algorithm:
        //Vector determining heading
        //Vector is composed of:
            //1. Attraction to unexplored frontier
            //2. Repulsion from other agents
            //3. Attraction to found target
            //4. Repulsion from objects/walls

}