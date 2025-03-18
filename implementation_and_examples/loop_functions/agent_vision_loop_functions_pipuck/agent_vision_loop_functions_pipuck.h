#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include "agent_implementation/feature_config.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "controllers/pipuck_hugo/pipuck_hugo.h"
//#include "agent_implementation/Quadtree.h"
#include <list>
using namespace argos;

class CAgentVisionLoopFunctions : public CLoopFunctions {

public:

    struct metrics {
        std::map<std::string, double> mission_time; //Per agent
        std::map<std::string, double> distance_to_deployment_site;
        std::map<std::string, std::vector<double>> coverage_over_time; //Per agent
        std::map<std::string, std::vector<double>> average_total_certainty_over_time; //Per agent, total certainty
        std::map<std::string, std::vector<double>> average_free_pheromone_over_time; //Per agent, certainty of presumed free space
        std::map<std::string, std::vector<double>> average_occupied_pheromone_over_time; //Per agent, certainty of presumed occupied space
        std::map<std::string, std::vector<std::pair<int, int>>> number_of_cells_and_leaves_over_time; //Per agent, Total number of cells, number of leaf nodes

        std::map<std::string, double> total_traveled_path; //Per agent
        std::map<std::string, double> total_battery_usage; //Per agent
        int n_agent_agent_collisions; //Make sure to divide by 2
        int n_agent_obstacle_collisions;
        std::vector<std::vector<int>> map_observation_count_total;
        std::map<std::string, std::vector<std::vector<int>>> map_observation_count; //Per agent
        std::map<std::string, std::pair<int, int>> bytes_sent_received; //Per agent
    };

    std::map<CPiPuckEntity*, bool> currently_colliding;
    int coverage_update_tick_interval = 160; //at 16 ticks/second, this is every 10 seconds
    std::map<CPiPuckEntity*, Coordinate> previous_positions;
    int nAgentsDone = 0;
    std::map<std::string, Coordinate> deployment_positions;

    metrics m_metrics;
    std::string metric_path_str = "experiment";

    std::vector<std::string> agents_relayed_map;
    std::vector<std::string> agents_returning;

    bool experimentFinished = false;
    float longest_mission_time_s = 0;

    typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TCoordinateMap;
    TCoordinateMap m_tObjectCoordinates;
    TCoordinateMap m_tOtherAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentCoordinates;
    std::map<CPiPuckEntity*, CRadians> m_tAgentHeadings;
    std::map<CPiPuckEntity*, CVector3> m_tAgentBestFrontierCoordinate;
    #ifdef USING_CONFIDENCE_TREE
    std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, double >>> m_tQuadTree;
    std::map<CPiPuckEntity*, std::vector<std::tuple<Coordinate, Coordinate>>> m_tNeighborPairs;
    #else
    std::map<CPiPuckEntity*, std::vector<std::vector<double>>> m_tCoverageMatrix;
    std::map<CPiPuckEntity*, std::vector<std::vector<double>>> m_tObstacleMatrix;
    #endif
    std::map<CPiPuckEntity*, double> m_tAgentElapsedTicks;
    double globalElapsedTime;
//    std::map<CPiPuckEntity*, std::vector<quadtree::Box>> m_tAgentFrontiers;
#ifdef USING_CONFIDENCE_TREE
    std::map<CPiPuckEntity*, std::vector<std::vector<std::pair<quadtree::Box, double>>>> m_tAgentFrontierRegions;
#else
    std::map<CPiPuckEntity*, std::vector<std::vector<Coordinate>>> m_tAgentFrontierRegions;
#endif
//    PheromoneMatrix combinedCoverageMatrix;
    std::map<CPiPuckEntity*, std::set<argos::CDegrees>> m_tAgentFreeAngles;
    #ifdef USING_CONFIDENCE_TREE
    std::vector<std::tuple<quadtree::Box, double>> combinedQuadTree;
    #else
    std::vector<std::vector<double>> coverageMatrix;
    int coverageMatrixWidth = 0;
    int coverageMatrixHeight = 0;
    double coverageMatrixResolution;
//    PheromoneMatrix combinedObstacleMatrix;
    std::vector<std::vector<double>> obstacleMatrix;
    int obstacleMatrixWidth = 0;
    int obstacleMatrixHeight = 0;
    double obstacleMatrixResolution;
    double real_x_min = -5.5;
    double real_y_min = -5.5;
    #endif
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

    #ifdef USING_CONFIDENCE_TREE
    inline const std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, double >>>& GetQuadTree() const {
        return m_tQuadTree;
    }
    #endif

    inline const std::map<CPiPuckEntity*, double>& GetAgentElapsedTicks() const {
        return m_tAgentElapsedTicks;
    }

    inline const std::map<CPiPuckEntity*, std::set<argos::CDegrees>> & GetAgentFreeAngles() const {
        return m_tAgentFreeAngles;
    }
#ifndef USING_CONFIDENCE_TREE
    Coordinate getRealCoordinateFromIndex(int x, int y, double resolution) const;
#endif


    static double calculateSpawnTime(double spawn_rate);

    std::list<CBoxEntity> spawn_boxes;
    double spawn_rate;
    std::list<int> spawn_times;



private:

//    void findAndPushObjectCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    void findAndPushOtherAgentCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);

    #ifdef USING_CONFIDENCE_TREE
    void pushQuadTreeIfTime(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void pushQuadTree(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    #else
    void pushMatricesIfTime(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void pushMatrices(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    #endif

    void updateCollisions(CPiPuckEntity *pcFB);
    void updateBatteryUsage(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void updateBytesSentReceived(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    #ifdef USING_CONFIDENCE_TREE
    void updateNumberOfCellsAndLeaves(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    void updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree);
    void updateCertainty(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree);
    void exportQuadtree(std::string filename);
    void exportQuadtree(std::string filename, CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent);
    #else
    void updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::vector<double>>& coverageMatrix, bool usingCoverageMatrix);
    void exportMatrices(std::string filename);
    void exportMatrices(std::string filename, CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent);
    #endif
    void updateTraveledPathLength(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    bool allAgentsDone(CSpace::TMapPerType &tFBMap);
    void checkReturnToDeploymentSite(CSpace::TMapPerType &tFBMap);
    void exportMetricsAndMaps();

    void updateCellObservationCount(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent);
    std::pair<int, int> coordinateToMapIndex(Coordinate coordinate, const std::shared_ptr<Agent> &agent);
    void observeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, const std::shared_ptr<Agent> &agent);
};

#endif
