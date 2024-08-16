#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include "agent_implementation/Quadtree.h"
using namespace argos;

class CAgentVisionLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TCoordinateMap;
    TCoordinateMap m_tObjectCoordinates;
    TCoordinateMap m_tOtherAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentBestFrontierCoordinate;
    std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, int, double >>> m_tQuadTree;
    std::map<CPiPuckEntity*, double> m_tAgentElapsedTicks;
    double globalElapsedTicks;

public:

   virtual ~CAgentVisionLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

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

    inline const std::map<CPiPuckEntity*, std::vector<std::tuple<quadtree::Box, int, double >>>& GetQuadTree() const {
        return m_tQuadTree;
    }

    inline const std::map<CPiPuckEntity*, double>& GetAgentElapsedTicks() const {
        return m_tAgentElapsedTicks;
    }

//    CBoxEntity* box = new CBoxEntity("new_box", CVector3(-2, 1, 0), CQuaternion(), false, CVector3(1.0, 1.0, 0.5), 0.0); ////        theMap.insert(std::make_pair("new_box", &box));

    std::tuple<argos::CVector3, argos::CVector3, int> spawnableObjects[10] = {
            std::make_tuple(argos::CVector3(-2, 1, 0), argos::CVector3(1.0, 1.0, 0.5), 10),
            std::make_tuple(argos::CVector3(2, 1, 0), argos::CVector3(1.0, 1.0, 0.5), 20),
            std::make_tuple(argos::CVector3(0, 1, 0), argos::CVector3(1.0, 1.0, 0.5), 30),
            std::make_tuple(argos::CVector3(-2, -1, 0), argos::CVector3(1.0, 1.0, 0.5), 40),
            std::make_tuple(argos::CVector3(2, -1, 0), argos::CVector3(1.0, 1.0, 0.5), 50),
            std::make_tuple(argos::CVector3(0, -1, 0), argos::CVector3(1.0, 1.0, 0.5), 60),
            std::make_tuple(argos::CVector3(-2, 0, 0), argos::CVector3(1.0, 1.0, 0.5), 70),
            std::make_tuple(argos::CVector3(2, 0, 0), argos::CVector3(1.0, 1.0, 0.5), 80),
            std::make_tuple(argos::CVector3(0, 0, 0), argos::CVector3(1.0, 1.0, 0.5), 90),
            std::make_tuple(argos::CVector3(0, 0, 0), argos::CVector3(1.0, 1.0, 0.5), 100)
    };

private:

    void findAndPushObjectCoordinates(CPiPuckEntity* pcFB, Agent* agent);
    void findAndPushOtherAgentCoordinates(CPiPuckEntity* pcFB, Agent* agent);
    void pushQuadTree(CPiPuckEntity* pcFB, Agent* agent);
};

#endif
