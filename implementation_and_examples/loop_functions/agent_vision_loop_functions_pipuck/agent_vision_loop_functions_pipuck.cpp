#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <set>
#include "agent_vision_loop_functions_pipuck.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include <chrono>

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

int loop_function_steps = 0;

std::chrono::time_point start = std::chrono::system_clock::now();

std::chrono::time_point end = std::chrono::system_clock::now();



/**
 * Get the coordinates in all occupied boxes of the quadtrees a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::findAndPushObjectCoordinates(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
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
void CAgentVisionLoopFunctions::findAndPushOtherAgentCoordinates(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    for (auto & it : agent->agentLocations) {
        Coordinate agentLocation = (it.second.first);
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
void CAgentVisionLoopFunctions::pushQuadTree(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    std::vector<std::tuple<quadtree::Box, float, double>> boxesAndConfidenceAndTicks = agent->quadtree->getAllBoxes();
    m_tNeighborPairs[pcFB] = agent->quadtree->getAllNeighborPairs();

    m_tQuadTree[pcFB] = boxesAndConfidenceAndTicks;
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
    Real ticksPerSecond = argos::CPhysicsEngine::GetInverseSimulationClockTick();
    /* Go through them */
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        agent->ticks_per_second = ticksPerSecond;
    }
}

/**
 * Clear all the vectors
 */

void CAgentVisionLoopFunctions::Reset() {
    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
    m_tAgentCoordinates.clear();
    m_tAgentBestFrontierCoordinate.clear();
    m_tQuadTree.clear();
    m_tAgentElapsedTicks.clear();
    m_tAgentFrontiers.clear();
    m_tAgentFrontierRegions.clear();

}

void CAgentVisionLoopFunctions::PreStep() {
    start = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = start-end;


    argos::LOG << "time between step: " << (elapsed_seconds.count()*1000) << "ms"
               << std::endl;

}


/**
 * Get the coordinates of all agents and objects in the environment
 */
void CAgentVisionLoopFunctions::PostStep() {
    auto temp_end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds_btwn = temp_end - end;

    argos::LOG << "time between post_step: " << (elapsed_seconds_btwn.count()*1000) << "ms"
               << std::endl;

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end-start;

    argos::LOG << "step time: " << (elapsed_seconds.count()*1000) << "ms"
              << std::endl;


    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
/* Go through them */
    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
    m_tAgentCoordinates.clear();
    m_tAgentBestFrontierCoordinate.clear();
    m_tQuadTree.clear();
    m_tAgentElapsedTicks.clear();
    m_tAgentFrontiers.clear();
    m_tAgentFrontierRegions.clear();
    m_tAgentFreeAngles.clear();
    m_tAgentSubTargetCoordinate.clear();
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        m_tAgentElapsedTicks[pcFB] = agent->elapsed_ticks/agent->ticks_per_second;
        globalElapsedTicks = agent->elapsed_ticks/agent->ticks_per_second;

        Coordinate bestFrontier = agent->currentBestFrontier.FromOwnToArgos();
        CVector3 bestFrontierPos = CVector3(bestFrontier.x, bestFrontier.y, 0.1f);
        m_tAgentBestFrontierCoordinate[pcFB] = bestFrontierPos;
        Coordinate subTarget = agent->subTarget.FromOwnToArgos();
        CVector3 subTargetPos = CVector3(subTarget.x, subTarget.y, 0.1f);
//        m_tAgentSubTargetCoordinate[pcFB] = subTargetPos;
#ifdef WALL_FOLLOWING_ENABLED
        Coordinate wallFollowingSubTarget = agent->wallFollower.wallFollowingSubTarget.FromOwnToArgos();
        CVector3 wallFollowingSubTargetPos = CVector3(wallFollowingSubTarget.x, wallFollowingSubTarget.y, 0.1f);
        m_tAgentWallFollowingSubTargetCoordinate[pcFB] = wallFollowingSubTargetPos;
#endif

        if(!agent->lineVisualization.empty()) m_tLine.clear();

        for(auto lineCoordinate: agent->lineVisualization){
            Coordinate line = lineCoordinate.FromOwnToArgos();
            CVector3 linePos = CVector3(line.x, line.y, 0.1f);
            m_tLine[pcFB].push_back(linePos);
        }


        findAndPushObjectCoordinates(pcFB, agent);
        findAndPushOtherAgentCoordinates(pcFB, agent);

        Coordinate pos = agent->position.FromOwnToArgos();
        CVector3 agentPos = CVector3(pos.x, pos.y, 0.03f);
        m_tAgentCoordinates[pcFB] = agentPos;
        pushQuadTree(pcFB, agent);

//        m_tAgentFrontiers[pcFB] = agent->current_frontiers;
        m_tAgentFrontierRegions[pcFB] = agent->current_frontier_regions;

//        m_tAgentFreeAngles[pcFB] = agent->freeAngles;
        for(auto angle: agent->freeAnglesVisualization) {
            m_tAgentFreeAngles[pcFB].insert(ToDegrees(Coordinate::OwnHeadingToArgos(ToRadians(angle))));
        }

    }

    auto mBox = quadtree::Box(-5, 5, 10);
//    std::unique_ptr<quadtree::Quadtree> combinedTree = std::make_unique<quadtree::Quadtree>(mBox);

    for (const auto & it : GetQuadTree()) {
//        for (std::tuple<quadtree::Box, float, double> boxesAndConfidenceAndTicks: it.second) {
//            quadtree::Box box = std::get<0>(boxesAndConfidenceAndTicks);
//            float LConfidence = std::get<1>(boxesAndConfidenceAndTicks);
//            double visitedTimeS = std::get<2>(boxesAndConfidenceAndTicks);
//
//            quadtree::QuadNode node{};
//            node.coordinate = box.getCenter();
//            node.LConfidence = LConfidence;
//            quadtree::Occupancy occ = quadtree::AMBIGUOUS;
//            if (node.LConfidence >= 0.4){
//                occ = quadtree::FREE;
//            } else if (node.LConfidence <= -0.85){
//                occ = quadtree::OCCUPIED;
//            }
//            node.occupancy = occ;
////            if (node.occupancy == quadtree::ANY || node.occupancy == quadtree::UNKNOWN)
////                continue;
//            node.visitedAtS = visitedTimeS;
//            combinedTree->add(node);
//        }

        if(it.first->GetId()=="pipuck1") combinedQuadTree = it.second;
    }
//    std::vector<std::tuple<quadtree::Box, float, double>> boxesAndConfidenceAndTicks = combinedTree->getAllBoxes();

//    combinedQuadTree = boxesAndConfidenceAndTicks;

//    CSpace::TMapPerType& theMap = GetSpace().GetEntitiesByType("box");
//    for(auto spawnObj: spawnableObjects) {
//        int spawn_time = std::get<2>(spawnObj);
//        if(loop_function_steps == spawn_time) {
//            std::unique_ptr<CBoxEntity> box = std::make_unique<CBoxEntity>("spawn_box" + std::to_string(spawn_time), std::get<0>(spawnObj), CQuaternion(), false, std::get<1>(spawnObj), 0.0);
//            GetSpace().AddEntity(*box);
//            CEmbodiedEntity *embodiedEntity = &box->GetEmbodiedEntity();
//            GetSpace().AddEntityToPhysicsEngine(*embodiedEntity);
//            box.release(); // Release ownership after adding to the space
//        }
//    }
//
    loop_function_steps++;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAgentVisionLoopFunctions, "agent_vision_loop_functions_pipuck")
