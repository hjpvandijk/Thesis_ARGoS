#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "controllers/pipuck_hugo/pipuck_hugo.h"
//#include "agent_implementation/Quadtree.h"
#include <set>
using namespace argos;

class CAgentVisionLoopFunctions : public CLoopFunctions {

public:

    typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TCoordinateMap;
    TCoordinateMap m_tObjectCoordinates;
    TCoordinateMap m_tOtherAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentBestFrontierCoordinate;
//    std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, int, double >>> m_tQuadTree;
    std::map<CPiPuckEntity*, std::vector<std::vector<double>>> m_tCoverageMatrix;
    std::map<CPiPuckEntity*, std::vector<std::vector<double>>> m_tObstacleMatrix;
    std::map<CPiPuckEntity*, double> m_tAgentElapsedTicks;
    double globalElapsedTicks;
//    std::map<CPiPuckEntity*, std::vector<quadtree::Box>> m_tAgentFrontiers;
    std::map<CPiPuckEntity*, std::vector<std::vector<Coordinate>>> m_tAgentFrontierRegions;
//    std::vector<std::tuple<quadtree::Box, int, double >> combinedQuadTree;
//    PheromoneMatrix combinedCoverageMatrix;
    std::map<CPiPuckEntity*, std::set<argos::CDegrees>> m_tAgentFreeAngles;
    std::vector<std::vector<double>> coverageMatrix;
    int coverageMatrixWidth;
    int coverageMatrixHeight;
    double coverageMatrixResolution;
//    PheromoneMatrix combinedObstacleMatrix;
    std::vector<std::vector<double>> obstacleMatrix;
    int obstacleMatrixWidth;
    int obstacleMatrixHeight;
    double obstacleMatrixResolution;

public:

    virtual ~CAgentVisionLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void PreStep();


    virtual void PostStep();

    inline const TCoordinateMap& GetObjectCoordinates() const {
        return m_tObjectCoordinates;
    }

    inline const TCoordinateMap& GetOtherAgentCoordinates() const {
        return m_tOtherAgentCoordinates;
    }

    inline const std::map<CPiPuckEntity*, CVector3>& GetAgentCoordinates() const {
        return m_tAgentCoordinates;
    }



    inline const std::map<CPiPuckEntity*, double>& GetAgentElapsedTicks() const {
        return m_tAgentElapsedTicks;
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

//    void findAndPushObjectCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    void findAndPushOtherAgentCoordinates(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);
    void pushQuadTree(CPiPuckEntity* pcFB, const std::shared_ptr<Agent>& agent);

};

#endif
