//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "utils/coordinate.h"
#include "agent_control/communication/simulation/radio.h"
#include "agent_control/motion/simulation/DifferentialDrive.h"
#include "utils/Quadtree.h"
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <set>
#include "agent_control/sensing/simulation/distance_sensor/hc_sr04.h"
#include "agent_implementation/agent_control/path_planning/WallFollower.h"
#include "agent_implementation/agent_control/path_planning/FrontierEvaluator.h"
#include "agent_implementation/agent_control/path_planning/ForceVectorCalculator.h"
#include "agent_control/battery/BatteryManager.h"
#include "agent_implementation/agent_control/path_planning/SimplePathPlanner.h"
#include "agent_implementation/agent_control/path_planning/PathFollower.h"

#define DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
//#define CLOSE_SMALL_AREAS
#define SEPARATE_FRONTIERS
//#define WALL_FOLLOWING_ENABLED
//#define BATTERY_MANAGEMENT_ENABLED
#define PATH_PLANNING_ENABLED
//#define AVOID_UNREACHABLE_FRONTIERS
#ifdef AVOID_UNREACHABLE_FRONTIERS
#ifndef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
#define DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
#endif
#endif
#define WALKING_STATE_WHEN_NO_FRONTIERS

class Agent {
public:
    std::string id{};
    Coordinate position{};
    argos::CRadians heading;
    argos::CRadians targetHeading;
    float speed{};

    //Differential drive
    DifferentialDrive differential_drive;

    //Radio
    Radio wifi;

    //Distance sensors
    static constexpr double num_sensors = 4;
    std::array<HC_SR04, static_cast<int>(num_sensors)> distance_sensors{};

#ifdef BATTERY_MANAGEMENT_ENABLED
    //Battery manager
    BatteryManager batteryManager;
#endif

    //Path planning engines
    WallFollower wallFollower;
#ifdef AVOID_UNREACHABLE_FRONTIERS
    FrontierEvaluator frontierEvaluator;
#endif
#ifdef PATH_PLANNING_ENABLED
    SimplePathPlanner pathPlanner;
    PathFollower pathFollower;
#endif


    double DISTANCE_SENSOR_NOISE_CM = 5.0;
    double ORIENTATION_NOISE_DEGREES = 5.0;
    double POSITION_NOISE_CM = 5.0;


    std::map<std::string, std::tuple<Coordinate, Coordinate, double>> agentLocations; //id: (location, frontier, timestamp)
    double AGENT_LOCATION_RELEVANT_DURATION_S = 10.0;
    double QUADTREE_EXCHANGE_INTERVAL_S = 5.0;
    std::map<std::string, double> agentQuadtreeSent; //id: sent timestamp
    std::map<std::string, int> agentQuadtreeBytesReceived; //id: bytes received
    std::map<std::string, std::pair<argos::CVector2, double>> agentVelocities; //id: (direction, speed)


    //Vector affected by swarm
    argos::CVector2 swarm_vector;
    //Force vector deciding the next position
    argos::CVector2 force_vector;


    Agent() {}

    explicit Agent(std::string id);

    void setPosition(double new_x, double new_y);

    void setPosition(Coordinate position);

    void setHeading(argos::CRadians new_heading);

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

    void sendQuadtreeToCloseAgents();
    void broadcastMessage(const std::string &message) const;
    void sendMessage(const std::string &message, const std::string& targetId) const;

    void checkMessages();

    void parseMessages();


    std::vector<std::string> getMessages();


    std::unique_ptr<quadtree::Quadtree> quadtree;



    double ticks_per_second = 30;




    double PROXIMITY_RANGE = 2.0;

    double TURN_THRESHOLD_DEGREES = 8.0;

    double AGENT_ROBOT_DIAMETER = 0.08;

    double OBJECT_SAFETY_RADIUS = 0.1;
    double AGENT_SAFETY_RADIUS = AGENT_ROBOT_DIAMETER + 0.1;

    double VIRTUAL_WALL_AVOIDANCE_WEIGHT = 1.1;
    double AGENT_COHESION_WEIGHT = 0;//0.23;
    double AGENT_AVOIDANCE_WEIGHT = 1.15;
    double AGENT_ALIGNMENT_WEIGHT = 0.5;//0.5;
    double UNEXPLORED_FRONTIER_WEIGHT = 0.3;

    double FRONTIER_DISTANCE_WEIGHT = 0.0;//0.001;
    double FRONTIER_SIZE_WEIGHT = 1.0;
    double FRONTIER_REACH_BATTERY_WEIGHT = 4.0;
    double FRONTIER_REACH_DURATION_WEIGHT = 0.5;

    double FRONTIER_SEARCH_DIAMETER = 8.0;

    double AGENT_COHESION_RADIUS = 1.5;
    double AGENT_AVOIDANCE_RADIUS = 0.68;
    double AGENT_ALIGNMENT_RADIUS = 1.5;
    double OBJECT_AVOIDANCE_RADIUS = AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS + 0.2;

#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    double FRONTIER_DIST_UNTIL_REACHED = OBJECT_AVOIDANCE_RADIUS;
#endif

#ifdef SEPARATE_FRONTIERS
    double FRONTIER_CLOSE_DISTANCE = 0.5;
#endif


    Coordinate left_right_borders = {-10, 10};
    Coordinate upper_lower_borders = {10, -10};

    float TURNING_SPEED_RATIO = 0.1;

    double ANGLE_INTERVAL_STEPS = 360;


#ifdef AVOID_UNREACHABLE_FRONTIERS
    int CLOSEST_COORDINATE_HIT_COUNT_BEFORE_DECREASING_CONFIDENCE = 3;
    int MAX_TICKS_IN_HITPOINT = int(ticks_per_second) * 5; //2 seconds
#endif

    double P_AVOIDANCE = 0.3; // 10% probability for avoidance to be correct
    double P_POSITION = 0.9; // 90% probability for position to be correct
    double P_FREE = 0.6; // 70% probability for free to be correct
    double P_OCCUPIED = 0.4; // 30% probability for occupied to be correct
    float ALPHA_RECEIVE = 0.1; // Factor with which a received value's probability is pulled towards 0.5
    double MIN_ALLOWED_DIST_BETWEEN_FRONTIERS = 1.0;

    float P_FREE_THRESHOLD = 0.6; //P > 0.6 means it is definitely free
    float P_OCCUPIED_THRESHOLD = 0.4; //P < 0.3 means it is definitely occupied


    Coordinate currentBestFrontier = {MAXFLOAT, MAXFLOAT};
    Coordinate previousBestFrontier = {0, 0};
    Coordinate subTarget = {MAXFLOAT, MAXFLOAT};


    uint32_t elapsed_ticks = 0;

    std::vector<quadtree::Box> current_frontiers;
    std::vector<std::vector<quadtree::Box>> current_frontier_regions;
    std::set<argos::CDegrees> freeAnglesVisualization;
    argos::CVector2 perpendicularVectorVisualization;
    std::vector<Coordinate> lineVisualization;
    std::vector<std::pair<Coordinate, Coordinate>> route_to_best_frontier;



private:
    void checkForObstacles();

    void checkIfAgentFitsBetweenObstacles(quadtree::Box obstacleBox) const;

    bool isObstacleBetween(Coordinate coordinate1, Coordinate coordinate2) const;

    argos::CVector2 calculateTotalVector(argos::CVector2 prev_total_vector,
                                         argos::CVector2 virtualWallAvoidanceVector,
                                         argos::CVector2 agentCohesionVector,
                                         argos::CVector2 agentAvoidanceVector,
                                         argos::CVector2 agentAlignmentVector,
                                         argos::CVector2 unexploredFrontierVector);


    std::vector<std::string> messages;

    std::string GetId() const;


    quadtree::Box addObjectLocation(Coordinate objectCoordinate, float Psensor) const;
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2, quadtree::Box objectBox, float Psensor) const;
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2, float Psensor) const;
    void addFreeAreaBetweenAndOccupiedAfter(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox, float Psensor) const;
    void addOccupiedAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;

    bool frontierPheromoneEvaporated();

#ifdef WALKING_STATE_WHEN_NO_FRONTIERS
    void enterWalkingState(argos::CVector2 & unexploredFrontierVector);
#endif



    };


#endif //THESIS_ARGOS_AGENT_H
