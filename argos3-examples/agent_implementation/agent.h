//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "coordinate.h"
#include "radio.h"
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>

class agent {
public:
    std::string id{};
    coordinate position;
    argos::CQuaternion heading;
    argos::CQuaternion targetHeading;
    double speed{};
    radio wifi;
    double lastRangeReading = 2;

    std::map<std::string, coordinate> agentLocations;

    argos::CCI_PiPuckDifferentialDriveActuator *diffdrive;


    //Distance sensor
    //Infrared sensor
    //DIfferential drive

    //Force vector deciding the next position
    argos::CVector2 force_vector;

    //Some sort of map or grid to keep track of the environment
    //Some sort of list of agents to keep track of other agents



    agent() {}

    explicit agent(std::string id);

    agent(std::string id, coordinate position);

    void setPosition(double new_x, double new_y);

    void setPosition(coordinate position);

    void setHeading(argos::CQuaternion new_heading);

    void setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *diffdrive);

    coordinate getPosition();

    std::string getId() const;

    void setId(std::string id);

    void setSpeed(double speed);

    double getSpeed() const;

    radio getWifi() const;

    void setWifi(radio wifi);

    void print();

    void updateMap();

    void setLastRangeReading(double new_range);

    void readDistanceSensor();

    void readInfraredSensor();

    void calculateNextPosition();

    void doStep();


    void broadcastMessage(std::string message);

    void checkMessages();

    void parseMessages();


    std::vector<std::string> getMessages();

private:
    argos::CQuaternion calculateObjectAvoidanceQT();

    argos::CQuaternion calculateAgentAvoidanceQT();

    std::vector<std::string> *messages;

    argos::CQuaternion averageQuaternions(std::vector<argos::CQuaternion> multipleRotations);


        std::string GetId() const;
};


#endif //THESIS_ARGOS_AGENT_H
