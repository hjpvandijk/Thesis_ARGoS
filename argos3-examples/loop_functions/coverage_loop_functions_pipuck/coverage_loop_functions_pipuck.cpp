#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <set>
#include "coverage_loop_functions_pipuck.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"

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

void CCoverageLoopFunctions::Init(TConfigurationNode &t_tree) {
    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);

        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

        CCI_PiPuckRangefindersSensor *rangefinders = cController.GetSensor<CCI_PiPuckRangefindersSensor>(
                "pipuck_rangefinders");
        double prox = 0.0f;
        std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> visitFn =
                [&prox](const CCI_PiPuckRangefindersSensor::SInterface &sensor) {
//                    range = std::get<3>(sensor.Configuration);
                    prox = sensor.Proximity;
                };
        rangefinders->Visit(visitFn);

        Agent *agent = cController.agentObject;

//       CVector3 pos = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
//       CVector3

        double opposite = argos::Sin(agent->heading) * agent->lastRangeReading;
        double adjacent = argos::Cos(agent->heading) * agent->lastRangeReading;


        Coordinate rayEnd = Coordinate{(agent->position.x + adjacent), (agent->position.y + opposite)}.FromOwnToArgos();

        CVector3 rayEnd3 = CVector3(rayEnd.x, rayEnd.y, 0.0f);

        /* Create a waypoint vector */
        m_tWaypoints[pcFB] = std::vector<CVector3>();
        /* Add the initial position of the pi-puck */
        m_tWaypoints[pcFB].push_back(rayEnd3);
        m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);

    }
}

/****************************************/
/****************************************/

void CCoverageLoopFunctions::Reset() {
    /*
     * Clear all the waypoint vectors
     */
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);

        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

        CCI_PiPuckRangefindersSensor *rangefinders = cController.GetSensor<CCI_PiPuckRangefindersSensor>(
                "pipuck_rangefinders");
        double prox = 0.0f;
        std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> visitFn =
                [&prox](const CCI_PiPuckRangefindersSensor::SInterface &sensor) {
//                    range = std::get<3>(sensor.Configuration);
                    prox = sensor.Proximity;
                };
        rangefinders->Visit(visitFn);

        Agent *agent = cController.agentObject;

//       CVector3 pos = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
//       CVector3

        double opposite = argos::Sin(agent->heading) * agent->lastRangeReading;
        double adjacent = argos::Cos(agent->heading) * agent->lastRangeReading;


        Coordinate rayEnd = Coordinate{(agent->position.x + adjacent), (agent->position.y + opposite)}.FromOwnToArgos();

        CVector3 rayEnd3 = CVector3(rayEnd.x, rayEnd.y, 0.0f);
        /* Clear the waypoint vector */
        m_tWaypoints[pcFB].clear();
        /* Add the initial position of the pi-puck */
        m_tWaypoints[pcFB].push_back(rayEnd3);
        m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);

    }
}

/****************************************/
/****************************************/

void CCoverageLoopFunctions::PostStep() {
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);

        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

        CCI_PiPuckRangefindersSensor *rangefinders = cController.GetSensor<CCI_PiPuckRangefindersSensor>(
                "pipuck_rangefinders");
        double prox = 0.0f;
        std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> visitFn =
                [&prox](const CCI_PiPuckRangefindersSensor::SInterface &sensor) {
//                    range = std::get<3>(sensor.Configuration);
                    prox = sensor.Proximity;
                };
        rangefinders->Visit(visitFn);

        Agent *agent = cController.agentObject;

//       CVector3 pos = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
//       CVector3

        double opposite = argos::Sin(agent->heading) * agent->lastRangeReading;
        double adjacent = argos::Cos(agent->heading) * agent->lastRangeReading;


        Coordinate rayEnd = Coordinate{(agent->position.x + adjacent), (agent->position.y + opposite)}.FromOwnToArgos();

        CVector3 rayEnd3 = CVector3(rayEnd.x, rayEnd.y, 0.0f);


        /* Add the current position of the pi-puck if it's sufficiently far from the last */
        if (SquareDistance(rayEnd3,
                           m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
            m_tWaypoints[pcFB].push_back(rayEnd3);
            m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);

        }
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCoverageLoopFunctions, "coverage_loop_functions_pipuck")
