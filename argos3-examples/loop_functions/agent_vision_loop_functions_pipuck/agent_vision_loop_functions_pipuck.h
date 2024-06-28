#ifndef AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include "agent_implementation/Quadtree.h"
using namespace argos;

class CAgentVisionLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TCoordinateMap;
    TCoordinateMap m_tObjectCoordinates;
    TCoordinateMap m_tOtherAgentCoordinates;
    std::map<CPiPuckEntity*, CVector3> m_tAgentCoordinates;
    std::map<CPiPuckEntity*, std::vector<std::pair<quadtree::Box, int>>> m_tQuadTree;

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

    inline const std::map<CPiPuckEntity*, std::vector<std::pair<quadtree::Box, int>>>& GetQuadTree() const {
        return m_tQuadTree;
    }

private:

    void findAndPushObjectCoordinates(CPiPuckEntity* pcFB, Agent* agent);
    void findAndPushOtherAgentCoordinates(CPiPuckEntity* pcFB, Agent* agent);
    void pushQuadTree(CPiPuckEntity* pcFB, Agent* agent);
};

#endif
