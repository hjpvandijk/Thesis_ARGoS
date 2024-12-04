//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "coordinate.h"
#include "radio.h"
#include "PheromoneMatrix.h"
//#include "Quadtree.h"
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <set>



class Agent {
public:
    std::string id{};
    Coordinate position{};
    argos::CRadians heading;
    argos::CRadians targetHeading;
    double speed{};
    Radio wifi{};
    static constexpr double num_sensors = 4;
    std::array<double, static_cast<int>(num_sensors)> lastRangeReadings{};


    std::map<std::string, Coordinate> agentLocations;
    std::map<std::string, std::pair<argos::CVector2, double>> agentVelocities;

    argos::CCI_PiPuckDifferentialDriveActuator *diffdrive{};


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

    void setPosition(double new_x, double new_y);

    void setPosition(Coordinate position);

    void setHeading(argos::CRadians new_heading);

    void setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *newDiffdrive);

    Coordinate getPosition() const;

    std::string getId() const;

    void setId(std::string id);

    void setSpeed(double speed);

    double getSpeed() const;

    Radio getWifi() const;

    void setWifi(Radio newWifi);

    void print() const;

    void updateMap();

    void setLastRangeReadings(int index, double new_range);

    void readDistanceSensor();

    void readInfraredSensor();

    void calculateNextPosition();

    void doStep();


    void broadcastMessage(const std::string& message) const;

    void checkMessages();

    void parseMessages();



    std::vector<std::string> getMessages();


//    std::unique_ptr<quadtree::Quadtree> quadtree;
    std::unique_ptr<PheromoneMatrix> coverageMatrix; //Cells that contain pheromone > 0, are covered and obstacle free
    std::unique_ptr<PheromoneMatrix> obstacleMatrix; //Cells that contain pheromone > 0, are covered and contain an obstacle

    double COVERAGE_MATRIX_RESOLUTION = 0.2;
    double OBSTACLE_MATRIX_RESOLUTION = 0.2;


    double PROXIMITY_RANGE = 2.0;

    double TURN_THRESHOLD_DEGREES = 2;

    double OBJECT_SAFETY_RADIUS = 0.1;
    double AGENT_SIZE = 0.08;
    double AGENT_SAFETY_RADIUS = AGENT_SIZE + 0.1;

    double VIRTUAL_WALL_AVOIDANCE_WEIGHT = 1.1;
    double AGENT_COHESION_WEIGHT = 0;//0.23;
    double AGENT_AVOIDANCE_WEIGHT = 1.15;
    double AGENT_ALIGNMENT_WEIGHT = 0.5;//0.5;
    double UNEXPLORED_FRONTIER_WEIGHT = 0.3;

    double FRONTIER_DISTANCE_WEIGHT = 0.2;//0.001;
    double FRONTIER_SIZE_WEIGHT = 1.0;

    double FRONTIER_SEARCH_DIAMETER = 8.0;

    double AGENT_COHESION_RADIUS = 1.5;
    double AGENT_AVOIDANCE_RADIUS = 2;
    double AGENT_ALIGNMENT_RADIUS = 1.5;
    double OBJECT_AVOIDANCE_RADIUS = OBJECT_SAFETY_RADIUS + AGENT_SAFETY_RADIUS + 0.2;

    Coordinate left_right_borders = {-10,10};
    Coordinate upper_lower_borders = {10,-10};

    double TURNING_SPEED_RATIO = 0.1;

    double ANGLE_INTERVAL_STEPS = 360;

    Coordinate currentBestFrontier = {0,0};

    double ticks_per_second = 30;
    uint32_t elapsed_ticks = 0;

    std::vector<Coordinate> current_frontiers;
    std::vector<std::vector<std::pair<int,int>>> current_frontier_regions;

    std::set<argos::CDegrees> freeAnglesVisualization;

private:
    void checkForObstacles();

    bool calculateObjectAvoidanceAngle(argos::CRadians* relativeObjectAvoidanceAngle, argos::CRadians targetAngle);
    argos::CVector2 getVirtualWallAvoidanceVector() const;
    bool getAverageNeighborLocation(Coordinate* averageNeighborLocation, double range);
    argos::CVector2 calculateAgentCohesionVector();
    argos::CVector2 calculateAgentAvoidanceVector();
    argos::CVector2 calculateAgentAlignmentVector();
    argos::CVector2 calculateUnexploredFrontierVector();
    std::vector<std::pair<int, int>> getFrontierCells(double currentTimeS);

    std::vector<std::string> messages;







    std::string GetId() const;


    void addObjectLocation(Coordinate objectCoordinate) const;
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;



};


#endif //THESIS_ARGOS_AGENT_H
