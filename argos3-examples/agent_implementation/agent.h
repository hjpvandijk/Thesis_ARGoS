//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "coordinate.h"
#include "radio.h"
#include "Quadtree.h"
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>




class Agent {
public:
    std::string id{};
    Coordinate position;
//    argos::CQuaternion heading;
    argos::CRadians heading;
    argos::CRadians targetHeading;
    double speed{};
    Radio wifi;
    double lastRangeReading = 2;

    std::map<std::string, Coordinate> agentLocations;
    std::map<std::string, std::pair<argos::CVector2, double>> agentVelocities;

    argos::CCI_PiPuckDifferentialDriveActuator *diffdrive;


    //Distance sensor
    //Infrared sensor
    //DIfferential drive

    //Vector affected by swarm
    argos::CVector2 swarm_vector;
    //Force vector deciding the next position
    argos::CVector2 force_vector;

    //Some sort of map or grid to keep track of the environment
    //Some sort of list of agents to keep track of other agents



    Agent() {}

    explicit Agent(std::string id);

    Agent(std::string id, Coordinate position);

    void setPosition(double new_x, double new_y);

    void setPosition(Coordinate position);

    void setHeading(argos::CRadians new_heading);

    void setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *diffdrive);

    Coordinate getPosition();

    std::string getId() const;

    void setId(std::string id);

    void setSpeed(double speed);

    double getSpeed() const;

    Radio getWifi() const;

    void setWifi(Radio wifi);

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


    quadtree::Quadtree *quadtree;


    double PROXIMITY_RANGE = 2.0;

    double TURN_THRESHOLD_DEGREES = 5;

    double OBJECT_AVOIDANCE_WEIGHT = 1;

    double AGENT_COHESION_WEIGHT = 0;//0.23;
    double AGENT_AVOIDANCE_WEIGHT = 1.15;
    double AGENT_ALIGNMENT_WEIGHT = 0;//0.5;
    double UNEXPLORED_FRONTIER_WEIGHT = 0.08;

    double FRONTIER_DISTANCE_WEIGHT = 1.0;//0.001;
    double FRONTIER_SIZE_WEIGHT = 1.0;

    double AGENT_COHESION_RADIUS = 1.5;
    double AGENT_AVOIDANCE_RANGE = 0.68;
    double AGENT_ALIGNMENT_RANGE = 1.5;

    double TURNING_SPEED_RATIO = 0.1;

    double ANGLE_INTERVAL_STEPS = 36;

    Coordinate currentBestFrontier = {0,0};

    double ticks_per_second = 10;
    uint32_t elapsed_ticks = 0;

private:
    void checkForObstacles();

    argos::CRadians calculateObjectAvoidanceVector();

    bool getAverageNeighborLocation(Coordinate* averageNeighborLocation);
    argos::CVector2 calculateAgentCohesionVector();
    argos::CVector2 calculateAgentAvoidanceVector(argos::CVector2 agentCohesionVector);
    argos::CVector2 calculateAgentAlignmentVector();
    argos::CVector2 calculateUnexploredFrontierVector();

    void updatePheromones();

    std::vector<std::string> *messages;







    std::string GetId() const;


    void addObjectLocation(Coordinate objectCoordinate);
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2);



};


#endif //THESIS_ARGOS_AGENT_H
