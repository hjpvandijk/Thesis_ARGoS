#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include "agent_implementation/feature_config.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include "agent_implementation/utils/Quadtree.h"
#include <list>
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
    std::map<CPiPuckEntity*, std::vector<std::pair<quadtree::Box, double>>> m_tAgentFrontiers;
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

//    inline const std::map<CPiPuckEntity*, std::vector<quadtree::Box>>& GetAgentFrontiers() const {
//        return m_tAgentFrontiers;
//    }

    inline const std::map<CPiPuckEntity*, std::vector<std::vector<std::pair<quadtree::Box, double>>>>& GetAgentFrontierRegions() const {
        return m_tAgentFrontierRegions;
    }

    inline const std::map<CPiPuckEntity*, std::set<argos::CDegrees>> & GetAgentFreeAngles() const {
        return m_tAgentFreeAngles;
    }

    std::list<CBoxEntity> spawn_boxes;
    std::list<int> spawn_times = {500, 800, 1200, 1400, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000, 10500, 11000, 11500, 12000, 12500, 13000, 13500, 14000, 14500, 15000, 15500, 16000, 16500, 17000, 17500, 18000, 18500, 19000, 19500, 20000, 20500, 21000, 21500, 22000, 22500, 23000, 23500, 24000, 24500, 25000, 25500, 26000, 26500, 27000, 27500, 28000, 28500, 29000, 29500, 30000, 30500, 31000, 31500, 32000, 32500, 33000, 33500, 34000, 34500, 35000, 35500, 36000, 36500, 37000, 37500, 38000, 38500, 39000, 39500, 40000, 40500, 41000, 41500, 42000, 42500, 43000, 43500, 44000, 44500, 45000, 45500, 46000, 46500, 47000, 47500, 48000, 48500, 49000, 49500, 50000, 50500, 51000, 51500, 52000, 52500, 53000, 53500, 54000, 54500, 55000, 55500, 56000, 56500, 57000, 57500, 58000, 58500, 59000, 59500, 60000, 60500, 61000, 61500, 62000, 62500, 63000, 63500, 64000, 64500};



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
