#include "trajectory_loop_functions_pipuck.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
   /* Get the map of all pi-pucks from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("pipuck");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current pi-puck */
      CPiPuckEntity* pcFB = any_cast<CPiPuckEntity*>(it->second);
      /* Create a waypoint vector */
      m_tWaypoints[pcFB] = std::vector<CVector3>();
      /* Add the initial position of the pi-puck */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all pi-pucks from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("pipuck");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current pi-puck */
      CPiPuckEntity* pcFB = any_cast<CPiPuckEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the pi-puck */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
   /* Get the map of all pi-pucks from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("pipuck");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current pi-puck */
       CPiPuckEntity* pcFB = any_cast<CPiPuckEntity*>(it->second);
      /* Add the current position of the pi-puck if it's sufficiently far from the last */
      if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions_pipuck")
