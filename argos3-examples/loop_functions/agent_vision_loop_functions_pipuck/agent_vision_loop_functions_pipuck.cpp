#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <set>
#include "agent_vision_loop_functions_pipuck.h"
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

/**
 * Get the coordinates in all occupied boxes of the quadtrees a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::findAndPushObjectCoordinates(CPiPuckEntity *pcFB, Agent *agent) {
    std::vector<quadtree::QuadNode> occupiedNodes = agent->quadtree->queryOccupied(agent->position,
                                                                                   agent->PROXIMITY_RANGE * 2.0);

    for (auto node: occupiedNodes) {
        Coordinate nodePos = node.coordinate.FromOwnToArgos();
        CVector3 pos = CVector3(nodePos.x, nodePos.y, 0.03f);
        m_tObjectCoordinates[pcFB].push_back(pos);
    }

}

/**
 * Get the coordinates of all other agents, received through messages
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::findAndPushOtherAgentCoordinates(CPiPuckEntity *pcFB, Agent *agent) {
    for (std::map<std::string, Coordinate>::const_iterator it = agent->agentLocations.begin();
         it != agent->agentLocations.end();
         ++it) {
        Coordinate agentLocation = (it->second);
        Coordinate agentLocationToArgos = agentLocation.FromOwnToArgos();
        CVector3 pos = CVector3(agentLocationToArgos.x, agentLocationToArgos.y, 0.02f);
        m_tOtherAgentCoordinates[pcFB].push_back(pos);
    }

}

/**
 * Get all the boxes and their occupancy from the quadtree of a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::pushQuadTree(CPiPuckEntity *pcFB, Agent *agent) {
    std::vector<std::tuple<quadtree::Box, int, double>> boxesAndOccupancyAndTicks = agent->quadtree->getAllBoxes();

    m_tQuadTree[pcFB] = boxesAndOccupancyAndTicks;
}

/****************************************/
/****************************************/

void CAgentVisionLoopFunctions::Init(TConfigurationNode &t_tree) {
    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    //Get ticks per second
    Real ticksPerSecond = GetSimulator().GetPhysicsEngines()[0]->GetInverseSimulationClockTick();
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);
        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        Agent *agent = cController.agentObject;

        agent->ticks_per_second = ticksPerSecond;
    }
}

/**
 * Clear all the vectors
 */

void CAgentVisionLoopFunctions::Reset() {
    /*
     * Clear all the waypoint vectors
     */
    /* Get the map of all pi-pucks from the space */
//    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    /* Go through them */
//    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
//         it != tFBMap.end();
//         ++it) {
//        /* Create a pointer to the current pi-puck */
//        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);
//
//        findAndPushObjectCoordinates(pcFB);
//
//
//    }

    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
}


/**
 * Get the coordinates of all agents and objects in the environment
 */
void CAgentVisionLoopFunctions::PostStep() {
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
/* Go through them */
    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
    m_tQuadTree.clear();
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it->second);
        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        Agent *agent = cController.agentObject;

        m_tAgentElapsedTicks[pcFB] = agent->elapsed_ticks/agent->ticks_per_second;

        Coordinate bestFrontier = agent->currentBestFrontier.FromOwnToArgos();
        CVector3 bestFrontierPos = CVector3(bestFrontier.x, bestFrontier.y, 0.1f);
        m_tAgentBestFrontierCoordinate[pcFB] = bestFrontierPos;



        findAndPushObjectCoordinates(pcFB, agent);
        findAndPushOtherAgentCoordinates(pcFB, agent);
//
        Coordinate pos = agent->position.FromOwnToArgos();
        CVector3 agentPos = CVector3(pos.x, pos.y, 0.03f);
        m_tAgentCoordinates[pcFB] = agentPos;
//        if(agent->getId() != "pipuck1") continue;
//
        pushQuadTree(pcFB, agent);


    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAgentVisionLoopFunctions, "agent_vision_loop_functions_pipuck")
