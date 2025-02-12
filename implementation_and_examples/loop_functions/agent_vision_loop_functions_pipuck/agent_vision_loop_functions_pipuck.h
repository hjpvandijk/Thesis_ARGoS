#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include "agent_implementation/feature_config.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include "agent_implementation/utils/Quadtree.h"
using namespace argos;

class CAgentVisionLoopFunctions : public CLoopFunctions {

public:

    struct metrics {
        std::map<std::string, double> mission_time; //Per agent
        std::map<std::string, bool> returned_to_deployment_site;
        std::map<std::string, std::vector<double>> coverage_over_time; //Per agent
        std::map<std::string, std::vector<double>> average_total_certainty_over_time; //Per agent, total certainty
        std::map<std::string, std::vector<double>> average_free_pheromone_over_time; //Per agent, certainty of presumed free space
        std::map<std::string, std::vector<double>> average_occupied_pheromone_over_time; //Per agent, certainty of presumed occupied space
        std::map<std::string, double> total_traveled_path; //Per agent
        std::map<std::string, double> total_battery_usage; //Per agent
        int n_agent_agent_collisions; //Make sure to divide by 2
        int n_agent_obstacle_collisions;
        std::vector<std::vector<int>> map_observation_count_total;
        std::map<std::string, std::vector<std::vector<int>>> map_observation_count; //Per agent
        std::map<std::string, std::pair<int, int>> bytes_sent_received; //Per agent
    };

    std::map<CPiPuckEntity*, bool> currently_colliding;
    int coverage_update_tick_interval = 300; //at 30 ticks/second, this is every 10 seconds
    std::map<CPiPuckEntity*, Coordinate> previous_positions;
    int nAgentsDone = 0;
    std::map<std::string, Coordinate> deployment_positions;

    metrics m_metrics;

    bool experimentFinished = false;
    float longest_mission_time_s = 0;

   typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TCoordinateMap;
    TCoordinateMap m_tObjectCoordinates;
    TCoordinateMap m_tOtherAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentCoordinates;
    std::map<CPiPuckEntity*, CRadians> m_tAgentHeadings;
    std::map<CPiPuckEntity*, CVector3> m_tAgentBestFrontierCoordinate;
    std::map<CPiPuckEntity*, CVector3> m_tAgentSubTargetCoordinate;
    std::map<CPiPuckEntity*, CVector3> m_tAgentWallFollowingSubTargetCoordinate;
    std::map<CPiPuckEntity*, CVector3> m_tPerpVector;
    std::map<CPiPuckEntity*, std::vector<CVector3>> m_tLine;



    std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, double >>> m_tQuadTree;
    std::map<CPiPuckEntity*, std::vector<std::tuple<Coordinate, Coordinate>>> m_tNeighborPairs;
    std::map<CPiPuckEntity*, double> m_tAgentElapsedTicks;
    double globalElapsedTime;
    std::map<CPiPuckEntity*, std::vector<quadtree::Box>> m_tAgentFrontiers;
    std::map<CPiPuckEntity*, std::vector<std::vector<std::pair<quadtree::Box, double>>>> m_tAgentFrontierRegions;
    std::map<CPiPuckEntity*, std::set<argos::CDegrees>> m_tAgentFreeAngles;
    std::vector<std::tuple<quadtree::Box, double>> combinedQuadTree;
    std::map<CPiPuckEntity*, std::vector<std::pair<Coordinate, Coordinate>>> m_tAgentRoute;
    std::map<CPiPuckEntity*, float> m_tAgentBatteryLevels;

public:

   virtual ~CAgentVisionLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

    virtual void PreStep();

    virtual void PostStep();

    virtual bool IsExperimentFinished();

    virtual void PostExperiment();

   inline const TCoordinateMap& GetObjectCoordinates() const {
      return m_tObjectCoordinates;
   }

    inline const TCoordinateMap& GetOtherAgentCoordinates() const {
        return m_tOtherAgentCoordinates;
    }

    inline const std::map<CPiPuckEntity*, CVector3>& GetAgentCoordinates() const {
        return m_tAgentCoordinates;
    }

    inline const std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, double >>>& GetQuadTree() const {
        return m_tQuadTree;
    }

    inline const std::map<CPiPuckEntity*, double>& GetAgentElapsedTicks() const {
        return m_tAgentElapsedTicks;
    }

    inline const std::map<CPiPuckEntity*, std::vector<quadtree::Box>>& GetAgentFrontiers() const {
        return m_tAgentFrontiers;
    }

    inline const std::map<CPiPuckEntity*, std::vector<std::vector<std::pair<quadtree::Box, double>>>>& GetAgentFrontierRegions() const {
        return m_tAgentFrontierRegions;
    }

    inline const std::map<CPiPuckEntity*, std::set<argos::CDegrees>> & GetAgentFreeAngles() const {
        return m_tAgentFreeAngles;
    }

//    CBoxEntity* box = new CBoxEntity("new_box", CVector3(-2, 1, 0), CQuaternion(), false, CVector3(1.0, 1.0, 0.5), 0.0); ////        theMap.insert(std::make_pair("new_box", &box));

//    <box id="box_6" size="1.41,0.97,0.5" movable="false">
//                                                 <body position="-1.585,1.385,0" orientation="0,0,0"/>
//                                                                                             </box>
//    <box id="box_7" size="4.07,0.37,0.5" movable="false">
//                                                 <body position="2.325,-3.465,0" orientation="0,0,0"/>
//                                                                                             </box>
//    <box id="box_8" size="2.76,0.32,0.5" movable="false">
//                                                 <body position="0.77,-4.61,0" orientation="0,0,0"/>
//                                                                                           </box>
//    <box id="box_9" size="0.16,1.09,0.5" movable="false">
//                                                 <body position="-1.03,-2.915,0" orientation="0,0,0"/>
//                                                                                             </box>
//    <box id="box_10" size="1.17,1.84,0.5" movable="false">
//                                                  <body position="-3.175,-2.27,0" orientation="0,0,0"/>
//                                                                                              </box>
    std::tuple<argos::CVector3, argos::CVector3, int> spawnableObjects[5] = {
//            std::make_tuple(argos::CVector3(-3, 1, 0), argos::CVector3(1.0, 1.0, 0.5), 100),
//            std::make_tuple(argos::CVector3(2.5, 1, 0), argos::CVector3(1.0, 1.0, 0.5), 400),
//            std::make_tuple(argos::CVector3(0, 4, 0), argos::CVector3(1.0, 1.0, 0.5), 600),
//            std::make_tuple(argos::CVector3(-2, -1, 0), argos::CVector3(1.0, 1.0, 0.5), 800),
//            std::make_tuple(argos::CVector3(2, -4, 0), argos::CVector3(1.0, 1.0, 0.5), 1000),
//            std::make_tuple(argos::CVector3(0, -1, 0), argos::CVector3(1.0, 1.0, 0.5), 1200),
    std::make_tuple(argos::CVector3(-1.585,1.385,0), argos::CVector3(1.41,0.97,0.5), 50),
    std::make_tuple(argos::CVector3(2.325,-3.465,0), argos::CVector3(4.07,0.37,0.5), 700),
    std::make_tuple(argos::CVector3(0.77,-4.61,0), argos::CVector3(2.76,0.32,0.5), 1000),
    std::make_tuple(argos::CVector3(-1.03,-2.915,0), argos::CVector3(0.16,1.09,0.5), 1200),
    std::make_tuple(argos::CVector3(-3.175,-2.27,0), argos::CVector3(1.17,1.84,0.5), 1500),


    };



private:

    void findAndPushObjectCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    void findAndPushOtherAgentCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    void pushQuadTree(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);

    void updateCollisions(CPiPuckEntity *pcFB);
    void updateBatteryUsage(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void updateBytesSentReceived(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree);
    void updateCertainty(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree);
    void updateTraveledPathLength(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    bool allAgentsDone(CSpace::TMapPerType &tFBMap);
    void updateAgentsFinishedTime(CSpace::TMapPerType &tFBMap);
    void checkReturnToDeploymentSite(CSpace::TMapPerType &tFBMap);
    void exportMetricsAndMaps();
    void updateCellObservationCount(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    std::pair<int, int> coordinateToMapIndex(Coordinate coordinate, const std::shared_ptr<Agent> &agent);
    void observeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, const std::shared_ptr<Agent> &agent);
    };

#endif
