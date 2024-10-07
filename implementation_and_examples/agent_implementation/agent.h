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




    std::map<std::string, std::pair<Coordinate, double>> agentLocations; //id: (location, timestamp)
    double AGENT_LOCATION_RELEVANT_DURATION_S = 10.0;
    std::map<std::string, bool> agentQuadtreeSent; //id: sent
    std::map<std::string, std::pair<argos::CVector2, double>> agentVelocities; //id: (direction, speed)

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

    void sendQuadtreeToCloseAgents();
    void broadcastMessage(const std::string &message) const;
    void sendMessage(const std::string &message, const std::string& targetId) const;

    void checkMessages();

    void parseMessages();


    std::vector<std::string> getMessages();


    std::unique_ptr<quadtree::Quadtree> quadtree;



    Coordinate left_right_borders = {-10, 10};
    Coordinate upper_lower_borders = {10, -10};

    double TURNING_SPEED_RATIO = 0.1;

    double ANGLE_INTERVAL_STEPS = 360;


#ifdef BLACKLIST_FRONTIERS
    std::map<Coordinate, std::pair<int, int>> blacklistedFrontiers; //coordinate: (count, currently avoiding)
    std::unique_ptr<quadtree::Quadtree> blacklistedTree; //Use quadtree for quick blacklisted frontier lookup
    double minDistFromFrontier = MAXFLOAT;


    Coordinate closestCoordinateToCurrentFrontier = {MAXFLOAT, MAXFLOAT};
    int closestCoordinateCounter = 0;
    bool lastTickInBlacklistHitPoint = false;
#endif

    Coordinate currentBestFrontier = {MAXFLOAT, MAXFLOAT};
    Coordinate previousBestFrontier = {0, 0};
    Coordinate subTarget = {MAXFLOAT, MAXFLOAT};

    int wallFollowingDirection = 0;

#ifdef WALL_FOLLOWING_ENABLED
    Coordinate wallFollowingSubTarget = {MAXFLOAT, MAXFLOAT};
    int prevWallFollowingDirection = 0;
    Coordinate wallFollowingHitPoint = {MAXFLOAT, MAXFLOAT};
    bool lastTickInWallFollowingHitPoint = false;
#endif

    double ticks_per_second = 30;
    uint32_t elapsed_ticks = 0;

    std::vector<quadtree::Box> current_frontiers;
    std::vector<std::vector<quadtree::Box>> current_frontier_regions;
    std::set<argos::CDegrees> freeAnglesVisualization;
    argos::CVector2 perpendicularVectorVisualization;
    std::vector<Coordinate> lineVisualization;


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


    bool calculateObjectAvoidanceAngle(argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle);

    argos::CVector2 getVirtualWallAvoidanceVector() const;

    bool getAverageNeighborLocation(Coordinate *averageNeighborLocation, double range);

    argos::CVector2 calculateAgentCohesionVector();

    argos::CVector2 calculateAgentAvoidanceVector();

    argos::CVector2 calculateAgentAlignmentVector();

    argos::CVector2 calculateUnexploredFrontierVector();

    std::vector<std::string> messages;


    std::string GetId() const;


    void addObjectLocation(Coordinate objectCoordinate) const;

    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;

    void addOccupiedAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;

    // Custom comparator to order set for wall following. The set is ordered by the angle difference to the wall following direction
    struct CustomComparator {
        int dir;  // dir is either 0, 1 or -1
        double heading;
        double targetAngle;

        CustomComparator(int dir, double heading, double targetAngle) : dir(dir), heading(heading),
                                                                        targetAngle(targetAngle) {}


        //SOMETHING GOES WRONG WITH ANGLE 122 AND HEADING 32 --> diff = 90 exactly
        //Good with heading 36 --> 86
        // Custom comparator logic
        bool operator()(const argos::CDegrees &a, const argos::CDegrees &b) const;
    };

#ifdef WALL_FOLLOWING_ENABLED
    void wallFollowing(const std::set<argos::CDegrees, CustomComparator>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle);
#endif
#ifdef BLACKLIST_FRONTIERS
    bool skipBlacklistedFrontier(double frontierRegionX, double frontierRegionY);
    void updateBlacklistFollowing(Coordinate bestFrontierRegionCenter);
    void resetBlacklistAvoidance(argos::CVector2 unexploredFrontierVector);
    bool closeToBlacklistedFrontier();
    void updateBlacklistChance();
#endif
#ifdef WALKING_STATE_WHEN_NO_FRONTIERS
    void enterWalkingState(argos::CVector2 & unexploredFrontierVector);
#endif



    };


#endif //THESIS_ARGOS_AGENT_H
