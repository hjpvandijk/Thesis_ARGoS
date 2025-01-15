//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "coordinate.h"
#include "PheromoneMatrix.h"
//#include "Quadtree.h"
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <set>
#include "agent_control/battery/BatteryManager.h"
#include "agent_control/motion/simulation/DifferentialDrive.h"
#include "agent_control/communication/simulation/Radio.h"


class Agent {
public:
    std::string id{};
    Coordinate position{};
    argos::CRadians heading;
    argos::CRadians targetHeading;
    float speed{};
    static constexpr double num_sensors = 4;
    std::array<double, static_cast<int>(num_sensors)> lastRangeReadings{};

    struct Config {
        float ROBOT_WEIGHT;
        float ROBOT_WHEEL_RADIUS;
        float ROBOT_INTER_WHEEL_DISTANCE;

        double OBJECT_SAFETY_RADIUS;
        double AGENT_SAFETY_RADIUS;

        double TURN_THRESHOLD_DEGREES;
        float TURNING_SPEED_RATIO;
        double STEPS_360_DEGREES;


        double DISTANCE_SENSOR_NOISE_CM;
        double ORIENTATION_NOISE_DEGREES;
        double POSITION_NOISE_CM;
        double DISTANCE_SENSOR_PROXIMITY_RANGE;

        double VIRTUAL_WALL_AVOIDANCE_WEIGHT;
        double AGENT_COHESION_WEIGHT;
        double AGENT_AVOIDANCE_WEIGHT;
        double AGENT_ALIGNMENT_WEIGHT;
        double UNEXPLORED_FRONTIER_WEIGHT;

        double FRONTIER_DISTANCE_WEIGHT;
        double FRONTIER_SIZE_WEIGHT;


        double FRONTIER_SEARCH_RADIUS;
//        int MAX_FRONTIER_CELLS;
//        int MAX_FRONTIER_REGIONS;
        double AGENT_COHESION_RADIUS;
        double AGENT_AVOIDANCE_RADIUS;
        double AGENT_ALIGNMENT_RADIUS ;
        double OBJECT_AVOIDANCE_RADIUS;

        double COVERAGE_MATRIX_RESOLUTION;
        double COVERAGE_MATRIX_EVAPORATION_TIME_S;
        double OBSTACLE_MATRIX_RESOLUTION;
        double OBSTACLE_MATRIX_EVAPORATION_TIME_S;

        double MATRIX_EXCHANGE_INTERVAL_S;

        double BATTERY_CAPACITY;
        double BATTERY_VOLTAGE;
        double MOTOR_STALL_CURRENT;
        double MOTOR_STALL_TORQUE;
        double MOTOR_NO_LOAD_RPM;
        double MOTOR_NO_LOAD_CURRENT;

        double WIFI_SPEED_MBPS;
        double MAX_JITTER_MS;
        double MESSAGE_LOSS_PROBABILITY;

    };
    Config config;

    std::map<std::string, std::pair<Coordinate, double>> agentLocations; //id: (location, timestamp)
     std::map<std::string, double> agentMatrixSent; //id: sent timestamp
    std::map<std::string, int> agentMatrixBytesReceived; //id: bytes received
    std::map<std::string, std::pair<argos::CVector2, double>> agentVelocities;

//    argos::CCI_PiPuckDifferentialDriveActuator *diffdrive{};
    DifferentialDrive differential_drive;

    Radio wifi;

    BatteryManager batteryManager;


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

//    void setDiffDrive(argos::CCI_PiPuckDifferentialDriveActuator *newDiffdrive);

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
    void sendMessage(const std::string &message, const std::string& targetId);

    void checkMessages();

    void parseMessages();



    std::vector<std::string> getMessages();


    std::unique_ptr<PheromoneMatrix> coverageMatrix; //Cells that contain pheromone > 0, are covered and obstacle free
    std::unique_ptr<PheromoneMatrix> obstacleMatrix; //Cells that contain pheromone > 0, are covered and contain an obstacle

    Coordinate left_right_borders = {-10,10};
    Coordinate upper_lower_borders = {10,-10};

    Coordinate currentBestFrontier = {0,0};

    double ticks_per_second = 30;
    uint32_t elapsed_ticks = 0;

    std::vector<Coordinate> current_frontiers;
    std::vector<std::vector<std::pair<int,int>>> current_frontier_regions;

    std::set<argos::CDegrees> freeAnglesVisualization;

private:
    std::string config_file = "agent_implementation/config.yaml";
    void loadConfig();
    void checkForObstacles();

    bool calculateObjectAvoidanceAngle(argos::CRadians* relativeObjectAvoidanceAngle, argos::CRadians targetAngle);
    argos::CVector2 getVirtualWallAvoidanceVector() const;
    bool getAverageNeighborLocation(Coordinate* averageNeighborLocation, double range);
    argos::CVector2 calculateAgentCohesionVector();
    argos::CVector2 calculateAgentAvoidanceVector();
    argos::CVector2 calculateAgentAlignmentVector();
    argos::CVector2 calculateUnexploredFrontierVector();
    std::vector<std::pair<int, int>> getFrontierCells(double currentTimeS, double searchRadius);

    std::vector<std::string> messages;







    std::string GetId() const;


    void addObjectLocation(Coordinate objectCoordinate) const;
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;



};


#endif //THESIS_ARGOS_AGENT_H
