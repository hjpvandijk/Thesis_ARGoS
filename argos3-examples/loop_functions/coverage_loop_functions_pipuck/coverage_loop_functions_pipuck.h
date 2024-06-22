#ifndef COVERAGE_LOOP_FUNCTIONS_PIPUCK_H
#define COVERAGE_LOOP_FUNCTIONS_PIPUCK_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>

using namespace argos;

class CCoverageLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CPiPuckEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   
public:

   virtual ~CCoverageLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

private:

};

#endif
