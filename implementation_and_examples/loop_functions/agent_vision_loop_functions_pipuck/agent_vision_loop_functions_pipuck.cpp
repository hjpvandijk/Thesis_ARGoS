#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include<argos3/plugins/robots/generic/simulator/simple_radios_default_actuator.h>
#include <set>
#include "agent_vision_loop_functions_pipuck.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include "agent_implementation/utils/Algorithms.h"
#include <chrono>
#include <sys/stat.h>
#include <cstdlib>  // for getenv()

/****************************************/
/****************************************/

//#define VISUALS

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
 * Get the coordinates of all other agents, received through messages
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::findAndPushOtherAgentCoordinates(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    for (auto & it : agent->agentLocations) {
        #ifdef SEPARATE_FRONTIERS
        Coordinate agentLocation = std::get<0>(it.second);
        #else
        Coordinate agentLocation = it.second.first;
        #endif
        Coordinate agentLocationToArgos = agentLocation.FromOwnToArgos();
        CVector3 pos = CVector3(agentLocationToArgos.x, agentLocationToArgos.y, 0.02f);
        m_tOtherAgentCoordinates[pcFB].push_back(pos);
    }

}

#ifdef USING_CONFIDENCE_TREE
/**
 * Get all the boxes and their occupancy from the quadtree of a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::pushQuadTreeIfTime(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    #ifndef VISUALS
    auto inMission = agent->state != Agent::State::NO_MISSION && agent->state != Agent::State::FINISHED_EXPLORING && agent->state != Agent::State::MAP_RELAYED;
    //Only have to update the quadtree every coverage_update_tick_interval ticks
    if (inMission && agent->elapsed_ticks % coverage_update_tick_interval == 0){
    #endif
        pushQuadTree(pcFB, agent);
    #ifndef VISUALS
    }
    #endif
}

/**
 * Get all the boxes and their occupancy from the quadtree of a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::pushQuadTree(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    std::vector<std::tuple<quadtree::Box, double>> boxesAndPheromones = agent->quadtree->getAllBoxes(globalElapsedTime);
//    m_tNeighborPairs[pcFB] = agent->quadtree->getAllNeighborPairs();

    m_tQuadTree[pcFB] = boxesAndPheromones;
}
#else

/**
 * Get all the boxes and their occupancy from the quadtree of a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::pushMatricesIfTime(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
    #ifndef VISUALS
    auto inMission = agent->state != Agent::State::NO_MISSION && agent->state != Agent::State::FINISHED_EXPLORING && agent->state != Agent::State::MAP_RELAYED;
    //Only have to update the quadtree every coverage_update_tick_interval ticks
    if (inMission && agent->elapsed_ticks % coverage_update_tick_interval == 0){
        #endif
        pushMatrices(pcFB, agent);
        #ifndef VISUALS
    }
    #endif
}

/**
 * Get all the boxes and their occupancy from the quadtree of a given agent
 * @param pcFB
 * @param agent
 */
void CAgentVisionLoopFunctions::pushMatrices(CPiPuckEntity *pcFB, const std::shared_ptr<Agent>& agent) {
//    std::vector<std::tuple<quadtree::Box, int, double>> boxesAndOccupancyAndTicks = agent->quadtree->getAllBoxes();
//    std::vector<std::tuple<quadtree::Box, int, double>> boxesAndOccupancyAndTicks = agent->quadtree->getAllBoxes();
    auto agentCoverageMatrix = agent->coverageMatrix->getMatrixPheromones(globalElapsedTime);
    auto agentObstacleMatrix = agent->obstacleMatrix->getMatrixPheromones(globalElapsedTime);
//    agent->quadtree->getAllBoxes(globalElapsedTime)

    m_tCoverageMatrix[pcFB] = agentCoverageMatrix;
    m_tObstacleMatrix[pcFB] = agentObstacleMatrix;

//    m_tQuadTree[pcFB] = boxesAndOccupancyAndTicks;
}
#endif

/****************************************/
/****************************************/

void CAgentVisionLoopFunctions::Init(TConfigurationNode &t_tree) {
    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    start = std::chrono::system_clock::now();

    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
    //Get ticks per second
    Real ticksPerSecond = argos::CPhysicsEngine::GetInverseSimulationClockTick();
    int map_cols = 0;
    int map_rows = 0;
    /* Go through them */
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);

        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        auto radio_entity = pcFB->GetComponentMap().find("simple_radios")->second;
        auto simple_radio = &(dynamic_cast<argos::CSimpleRadioEquippedEntity*>(radio_entity))->GetRadio(0);
        simple_radio->SetRange(agent->config.WIFI_RANGE_M);

        assert(agent->ticks_per_second == ticksPerSecond);

        #ifndef USING_CONFIDENCE_TREE
        real_x_min = - agent->coverageMatrix->getRealWidth()/ 2;
        real_y_min = - agent->coverageMatrix->getRealHeight()/ 2;
        #endif

        currently_colliding[pcFB] = false;
        previous_positions[pcFB] = Coordinate{MAXFLOAT, MAXFLOAT};
        #ifdef USING_CONFIDENCE_TREE
        map_cols = std::max(map_cols, int(agent->quadtree->getRootBox().getSize()/agent->quadtree->getResolution()));
        map_rows = std::max(map_rows, int(agent->quadtree->getRootBox().getSize()/agent->quadtree->getResolution()));
        #else
        map_cols = std::max(map_cols, std::max(agent->coverageMatrix->getWidth(), agent->obstacleMatrix->getWidth()));
        map_rows = std::max(map_rows, std::max(agent->coverageMatrix->getHeight(), agent->obstacleMatrix->getHeight()));
        #endif
        Coordinate agentRealPosition = cController.getActualAgentPosition();
        deployment_positions[pcFB->GetId()] = agentRealPosition;
    }

    this->m_metrics = metrics();
    this->m_metrics.map_observation_count_total.resize(map_cols, std::vector<int>(map_rows, 0));
    this->m_metrics.map_observation_count = std::map<std::string, std::vector<std::vector<int>>>();
    for (const auto& it : tFBMap) {
        this->m_metrics.map_observation_count[it.first] = std::vector<std::vector<int>>(map_cols, std::vector<int>(map_rows, 0));
    }

    const char* seed = std::getenv("SEED");
    int seed_int = seed ? std::stoi(seed) : 0;
    srand(seed_int);

    const char* average_inter_spawn_time_s = std::getenv("AVERAGE_INTER_SPAWN_TIME");
    float average_inter_spawn_time = std::stof(average_inter_spawn_time_s);
    this->spawn_rate = average_inter_spawn_time_s ? 1.0f /  average_inter_spawn_time: 10.0f;
    argos::LOG << "Average inter spawn time: " << average_inter_spawn_time << std::endl;

    CSpace::TMapPerType &boxMap = GetSpace().GetEntitiesByType("box");
    double prev_spawn_time = 0.0f;
    for (auto &it: boxMap) {
        CBoxEntity *box = any_cast<CBoxEntity *>(it.second);
        if (box->GetId().find("spawn_box") != std::string::npos) {
            argos::LOG << "Found spawn box: " << box->GetId() << std::endl;
            if (average_inter_spawn_time > 0) { //If the spawn time is 0, don't spawn any boxes
                auto copybox = new CBoxEntity(box->GetId(), box->GetEmbodiedEntity().GetOriginAnchor().Position,
                                              box->GetEmbodiedEntity().GetOriginAnchor().Orientation, false,
                                              box->GetSize(), 1.0f);
                int spawn_time_ticks = int(calculateSpawnTime(this->spawn_rate) * ticksPerSecond) + prev_spawn_time;
                prev_spawn_time = spawn_time_ticks;
                this->spawn_boxes.push_back(*copybox);
                this->spawn_times.push_back(spawn_time_ticks);
                argos::LOG << "Spawn time: " << spawn_time_ticks << std::endl;
                delete copybox;
            }
            RemoveEntity(*box);
        }
    }

    const char* metric_path = std::getenv("METRIC_PATH");
    if (metric_path) {
        //Remove ".argos"
        this->metric_path_str = metric_path;
        this->metric_path_str = metric_path_str.substr(0, metric_path_str.find_last_of('.'));
    }
}

double CAgentVisionLoopFunctions::calculateSpawnTime(double spawn_rate){
    auto val = -std::log(1.0f - (float(rand()-1)/float(RAND_MAX))) / spawn_rate;
    return val;
}

/**
 * Clear all the vectors
 */

void CAgentVisionLoopFunctions::Reset() {
    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
    m_tAgentCoordinates.clear();
    m_tAgentBestFrontierCoordinate.clear();
    #ifdef USING_CONFIDENCE_TREE
    m_tQuadTree.clear();
    #else
    m_tCoverageMatrix.clear();
    m_tObstacleMatrix.clear();
    #endif
    m_tAgentElapsedTicks.clear();
//    m_tAgentFrontiers.clear();
    m_tAgentFrontierRegions.clear();
    m_tAgentFreeAngles.clear();

}

void CAgentVisionLoopFunctions::PreStep() {
//    start = std::chrono::system_clock::now();
//
//    std::chrono::duration<double> elapsed_seconds = start-end;
//
//
//    argos::LOG << "time between step: " << (elapsed_seconds.count()*1000) << "ms"
//               << std::endl;

}

#ifndef USING_CONFIDENCE_TREE
Coordinate CAgentVisionLoopFunctions::getRealCoordinateFromIndex(int x, int y, double resolution) const {
    //Get the real world coordinates from the matrix coordinates
    double x_real = this->real_x_min + x * resolution;
    double y_real = this->real_y_min + y * resolution;
    return {x_real + resolution/2, y_real + resolution/2};
}
#endif


/**
 * Get the coordinates of all agents and objects in the environment
 */
void CAgentVisionLoopFunctions::PostStep() {
//    auto temp_end = std::chrono::system_clock::now();
//
//    std::chrono::duration<double> elapsed_seconds_btwn = temp_end - end;
//
//    argos::LOG << "time between post_step: " << (elapsed_seconds_btwn.count()*1000) << "ms"
//               << std::endl;
//
//    end = std::chrono::system_clock::now();
//
//    std::chrono::duration<double> elapsed_seconds = end-start;
//
//    argos::LOG << "step time: " << (elapsed_seconds.count()*1000) << "ms"
//               << std::endl;

//
    /* Get the map of all pi-pucks from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("pipuck");
/* Go through them */
    m_tObjectCoordinates.clear();
    m_tOtherAgentCoordinates.clear();
    m_tAgentCoordinates.clear();
    m_tAgentBestFrontierCoordinate.clear();
#ifdef USING_CONFIDENCE_TREE
    m_tQuadTree.clear();
#else
    m_tCoverageMatrix.clear();
    m_tObstacleMatrix.clear();
#endif
    m_tAgentElapsedTicks.clear();
//    m_tAgentFrontiers.clear();
    m_tAgentFrontierRegions.clear();
    m_tAgentFreeAngles.clear();
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        m_tAgentElapsedTicks[pcFB] = agent->elapsed_ticks/agent->ticks_per_second;
        globalElapsedTime = agent->elapsed_ticks / agent->ticks_per_second;
        #ifdef VISUALS

        Coordinate bestFrontier = agent->currentBestFrontier.FromOwnToArgos();
        CVector3 bestFrontierPos = CVector3(bestFrontier.x, bestFrontier.y, 0.1f);
        m_tAgentBestFrontierCoordinate[pcFB] = bestFrontierPos;



//        findAndPushObjectCoordinates(pcFB, agent);
        findAndPushOtherAgentCoordinates(pcFB, agent);

        Coordinate pos = agent->position.FromOwnToArgos();
        CVector3 agentPos = CVector3(pos.x, pos.y, 0.03f);
        m_tAgentCoordinates[pcFB] = agentPos;
        m_tAgentHeadings[pcFB] = Coordinate::OwnHeadingToArgos(agent->heading);
        #ifdef USING_CONFIDENCE_TREE
        m_tAgentFrontierRegions[pcFB] = agent->current_frontier_regions;
        #else

//        m_tAgentFrontiers[pcFB] = agent->current_frontiers;
        for (auto regioncenter : agent->current_frontier_regions){
            std::vector<Coordinate> regionCoordinates;
            for (auto cellIndex: regioncenter) {
                auto realCellCoordinate = getRealCoordinateFromIndex(cellIndex.first, cellIndex.second,
                                                                     agent->coverageMatrix->getResolution());
                regionCoordinates.push_back(realCellCoordinate);
            }
            m_tAgentFrontierRegions[pcFB].push_back(regionCoordinates);
        }
        #endif


        for(auto angle: agent->freeAnglesVisualization) {
            m_tAgentFreeAngles[pcFB].insert(ToDegrees(Coordinate::OwnHeadingToArgos(ToRadians(angle))));
        }
        #endif
        m_tAgentBatteryLevels[pcFB] = agent->batteryManager.battery.getStateOfCharge() * 100.0f;

        #ifdef USING_CONFIDENCE_TREE
        pushQuadTreeIfTime(pcFB, agent);
        updateNumberOfCellsAndLeaves(pcFB, agent);
        #else
        pushMatricesIfTime(pcFB, agent);
        #endif

        updateCollisions(pcFB);
        updateTraveledPathLength(pcFB, agent);
        updateBatteryUsage(pcFB, agent);
        updateCellObservationCount(pcFB, agent);
        updateBytesSentReceived(pcFB, agent);

    }

//    auto mBox = quadtree::Box(-5, 5, 10);
//    std::unique_ptr<quadtree::Quadtree> combinedTree = std::make_unique<quadtree::Quadtree>(mBox);
//
#ifdef USING_CONFIDENCE_TREE
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
//        updateCoverage(it.first, it.second);
//        updateCertainty(it.first, it.second);
        updateCoverage(it.first, it.second);
        updateCertainty(it.first, it.second);
        if(it.first->GetId()=="pipuck1") combinedQuadTree = it.second;
    }
#else
//    combinedCoverageMatrix = PheromoneMatrix(10, 10, 0.2);


    for (const auto& it : m_tCoverageMatrix) {
//        double** coverageMatrix = it.second;
//        int height = sizeof *coverageMatrix / sizeof coverageMatrix[0]; // rows
//        int width = sizeof *coverageMatrix[0] / sizeof(double); // columns
//        for (int i = 0; i < width; i++) {
//            for (int j = 0; j < height; j++) {
//                combinedCoverageMatrix.update(i, j, coverageMatrix[i][j]);
//            }
//        }
        auto obstacleMatrixOfAgent = m_tObstacleMatrix[it.first];

//#ifdef VISUALS
        if(it.first->GetId() == "pipuck2"){
            coverageMatrix = it.second;
            coverageMatrixWidth = coverageMatrix.size();
            coverageMatrixHeight = coverageMatrix[0].size(); // columns;
            coverageMatrixResolution = -(real_x_min * 2) / coverageMatrixWidth;
//            argos::LOG << "setting" << std::endl;
            obstacleMatrix = obstacleMatrixOfAgent;
            obstacleMatrixWidth = obstacleMatrix.size();
            obstacleMatrixHeight = obstacleMatrix[0].size(); // columns;
            obstacleMatrixResolution = -(real_x_min * 2) / obstacleMatrixWidth;
        }
//#endif
        updateCoverage(it.first, it.second, obstacleMatrixOfAgent);
    }
//    combinedObstacleMatrix = PheromoneMatrix(10, 10, 0.2);


#endif


    if (!this->spawn_boxes.empty()) {
        auto spawn_time_front = this->spawn_times.front();
        if (loop_function_steps >= spawn_time_front) {
            argos::LOG << "Spawn box at: " << loop_function_steps << std::endl;
            argos::LOG << "Spawning box at " << spawn_time_front << std::endl;
            auto box = this->spawn_boxes.front();
            auto newBox = new CBoxEntity(box.GetId(), box.GetEmbodiedEntity().GetOriginAnchor().Position, box.GetEmbodiedEntity().GetOriginAnchor().Orientation, false, box.GetSize(), 1.0f);
//            argos::LOG << "Spawning box " << box.GetId() << " at " << box.GetEmbodiedEntity().GetOriginAnchor().Position << std::endl;
            AddEntity(*newBox);
            this->spawn_times.pop_front();
            this->spawn_boxes.pop_front();
        }
    }

    //If a new agent is done, update the metrics and maps
    if (allAgentsDone(tFBMap)) {
        checkReturnToDeploymentSite(tFBMap);
        //Update quadtree for the last time
        for (auto & it : tFBMap) {
            CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
            auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
            std::shared_ptr<Agent> agent = cController.agentObject;
            #ifdef USING_CONFIDENCE_TREE
            pushQuadTree(pcFB, agent);
            #else
            pushMatrices(pcFB, agent);
            #endif
        }
        exportMetricsAndMaps();
        experimentFinished = true;
    }

    loop_function_steps++;
}

#ifdef USING_CONFIDENCE_TREE
void CAgentVisionLoopFunctions::updateNumberOfCellsAndLeaves(argos::CPiPuckEntity *pcFB,
                                                             const std::shared_ptr<Agent> &agent) {
    //Get controller
    auto inMission = agent->state != Agent::State::NO_MISSION && agent->state != Agent::State::FINISHED_EXPLORING && agent->state != Agent::State::MAP_RELAYED;
    //Update coverage over time at every interval, if mission has started
//    argos::LOG << "Updating coverage for " << pcFB->GetId() << " which is inmission: " << inMission << std::endl;
    if (inMission && agent->elapsed_ticks % coverage_update_tick_interval == 0) {
        this->m_metrics.number_of_cells_and_leaves_over_time[pcFB->GetId()].push_back(std::make_pair(agent->quadtree->numberOfCells,
                                                                                                     agent->quadtree->numberOfLeafNodes));
    }
}
#endif

void CAgentVisionLoopFunctions::updateCollisions(CPiPuckEntity *pcFB) {
//Get collisions
    auto collisions = pcFB->GetEmbodiedEntity().IsCollidingWithWhat();
    if (collisions != nullptr) {
        if (currently_colliding[pcFB]) return; //Only count once per collision
        auto collisionVec =  static_cast<std::vector<void*>*>(collisions);
        for (auto collision: *collisionVec) {
            auto collisionType = static_cast<std::string*>(collision);
            if (*collisionType == "pipuck") {
                m_metrics.n_agent_agent_collisions++;
            } else { //We consider all other collisions as obstacle collisions
                m_metrics.n_agent_obstacle_collisions++;
            }
        }
        currently_colliding[pcFB] = true;
    } else {
        currently_colliding[pcFB] = false;
    }

}

#ifdef USING_CONFIDENCE_TREE
void CAgentVisionLoopFunctions::updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree) {
    //Get controller
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
    auto inMission = cController.agentObject->state != Agent::State::NO_MISSION && cController.agentObject->state != Agent::State::FINISHED_EXPLORING && cController.agentObject->state != Agent::State::MAP_RELAYED;
    //Update coverage over time at every interval, if mission has started
//    argos::LOG << "Updating coverage for " << pcFB->GetId() << " which is inmission: " << inMission << std::endl;
    if (inMission && cController.agentObject->elapsed_ticks % coverage_update_tick_interval == 0){
//        argos::LOG << "Updating coverage for " << pcFB->GetId() << std::endl;

        double covered_area = 0;


        for (auto &it: tree) {
            quadtree::Box box = std::get<0>(it);

            double box_size = box.getSize();
            covered_area += box_size * box_size;
        }

        double coverage = covered_area;
//        argos::LOG << "[" << pcFB->GetId() << "] Coverage: " << coverage << std::endl;


        m_metrics.coverage_over_time[pcFB->GetId()].push_back(coverage);
    }
}

void CAgentVisionLoopFunctions::updateCertainty(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double>> &tree) {
    //Get controller
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
    auto inMission = cController.agentObject->state != Agent::State::NO_MISSION && cController.agentObject->state != Agent::State::FINISHED_EXPLORING && cController.agentObject->state != Agent::State::MAP_RELAYED;
    double resolution = cController.agentObject->quadtree->getResolution();
        //Update certainty over time at every interval, if mission has started
    if (inMission && cController.agentObject->elapsed_ticks % coverage_update_tick_interval == 0){
        double total_certainty = 0;
        double total_certainty_areacompensated = 0;
        int total_boxes = 0;
        double total_box_area = 0;
        double free_certainty = 0;
        double free_certainty_areacompensated = 0;
        int free_boxes = 0;
        double free_box_area = 0;
        double occupied_certainty = 0;
        double occupied_certainty_areacompensated = 0;
        int occupied_boxes = 0;
        double occupied_box_area = 0;

        for (auto & it : tree) {
            auto box = std::get<0>(it);
            auto pheromone = std::get<1>(it);
            total_certainty += std::abs(pheromone-0.5);
            total_certainty_areacompensated += std::abs(pheromone-0.5) * box.getSize()*box.getSize();
            total_boxes++;
            total_box_area += box.getSize() * box.getSize();
            if (pheromone >= 0.5) {
                free_certainty += pheromone;
                free_certainty_areacompensated += pheromone * box.getSize()*box.getSize();
                free_boxes++;
                free_box_area += box.getSize() * box.getSize();
            } else if (pheromone < 0.5) {
                occupied_certainty += pheromone;
                occupied_certainty_areacompensated += pheromone * box.getSize()*box.getSize();
                occupied_boxes++;
                occupied_box_area += box.getSize() * box.getSize();
            }
        }

        double average_total_certainty = total_certainty / total_boxes;
        double average_free_certainty = free_certainty / free_boxes;
        double average_occupied_certainty = occupied_certainty / occupied_boxes;
//        argos::LOG << "[" << pcFB->GetId() << "] Average total certainty: " << average_total_certainty << std::endl;
//        argos::LOG << "[" << pcFB->GetId() << "] Average free certainty: " << average_free_certainty << std::endl;
//        argos::LOG << "[" << pcFB->GetId() << "] Average occupied certainty: " << average_occupied_certainty << std::endl;


        m_metrics.average_total_certainty_over_time[pcFB->GetId()].push_back(average_total_certainty);
        m_metrics.average_free_pheromone_over_time[pcFB->GetId()].push_back(average_free_certainty);
        m_metrics.average_occupied_pheromone_over_time[pcFB->GetId()].push_back(average_occupied_certainty);

        double average_total_certainty_areacompensated = total_certainty_areacompensated / total_box_area;
        double average_free_certainty_areacompensated = free_certainty_areacompensated / free_box_area;
        double average_occupied_certainty_areacompensated = occupied_certainty_areacompensated / occupied_box_area;

        m_metrics.average_total_certainty_over_time_areacompensated[pcFB->GetId()].push_back(average_total_certainty_areacompensated);
        m_metrics.average_free_pheromone_over_time_areacompensated[pcFB->GetId()].push_back(average_free_certainty_areacompensated);
        m_metrics.average_occupied_pheromone_over_time_areacompensated[pcFB->GetId()].push_back(average_occupied_certainty_areacompensated);


    }

}
#else
void CAgentVisionLoopFunctions::updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::vector<double>> &coverageMatrix, const std::vector<std::vector<double>> &obstacleMatrix) {
    //Get controller
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
    auto inMission = cController.agentObject->state != Agent::State::NO_MISSION && cController.agentObject->state != Agent::State::FINISHED_EXPLORING && cController.agentObject->state != Agent::State::MAP_RELAYED;
    //Update coverage over time at every interval, if mission has started
    if (inMission && cController.agentObject->elapsed_ticks % coverage_update_tick_interval == 0){

        double covered_area = 0;


        for (int i = 0; i < coverageMatrix.size(); i++) {
            for (int j = 0; j < coverageMatrix[i].size(); j++) {
                if (coverageMatrix[i][j] > 0) { //Will be 0 when not visited or covered by obstacle matrix
                    covered_area += cController.agentObject->coverageMatrix->getResolution() *
                                    cController.agentObject->coverageMatrix->getResolution();
                }
            }
        }

        for (int i = 0; i < obstacleMatrix.size(); i++) {
            for (int j = 0; j < obstacleMatrix[i].size(); j++) {
                if (obstacleMatrix[i][j] > 0) { //Will be 0 when not visited
                    covered_area += cController.agentObject->obstacleMatrix->getResolution() *
                                    cController.agentObject->obstacleMatrix->getResolution();
                }
            }
        }


        double coverage = covered_area;
//        argos::LOG << "[" << pcFB->GetId() << "] Coverage: " << coverage << std::endl;


        m_metrics.coverage_over_time[pcFB->GetId()].push_back(coverage);
    }
}

#endif
void CAgentVisionLoopFunctions::updateTraveledPathLength(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

    if (previous_positions[pcFB].x != MAXFLOAT) {

        double distance = sqrt(pow(cController.getActualAgentPosition().x - previous_positions[pcFB].x, 2) + pow(cController.getActualAgentPosition().y - previous_positions[pcFB].y, 2));
        m_metrics.total_traveled_path[pcFB->GetId()] += distance;
    }

    previous_positions[pcFB] = cController.getActualAgentPosition();
//    argos::LOG << "[" << pcFB->GetId() << "] Traveled path: " << m_metrics.total_traveled_path[pcFB->GetId()] << std::endl;
}

void CAgentVisionLoopFunctions::exportMetricsAndMaps() {
    //Create directory 'experiment_results' if it does not exist
    std::string dir = "experiment_results";
    if (mkdir(dir.c_str(), 0777) == -1) {
        std::cerr << "Error :  " << strerror(errno) << std::endl;
    }


    if (mkdir(metric_path_str.c_str(), 0777) == -1) {
        std::cerr << "Error :  " << strerror(errno) << std::endl;
    }

    argos::LOG << "Exporting metrics and maps to " << metric_path_str << std::endl;


    //Export metrics
    std::ofstream metricsFile;
    metricsFile.open( metric_path_str + "/metrics.csv");
    metricsFile << "n_agent_agent_collisions,";
    metricsFile << "n_agent_obstacle_collisions\n";
    metricsFile << m_metrics.n_agent_agent_collisions << ",";
    metricsFile << m_metrics.n_agent_obstacle_collisions << "\n";
    metricsFile.close();

    //Export agent returned to deployment site
    std::ofstream distanceToDeploymentSiteFile;
    distanceToDeploymentSiteFile.open(metric_path_str + "/distance_to_deployment_site.csv");
    distanceToDeploymentSiteFile << "agent_id,distance_to_deployment_site\n";
    for (auto & it : m_metrics.distance_to_deployment_site) {
        distanceToDeploymentSiteFile << it.first << "," << it.second << "\n";
    }
    distanceToDeploymentSiteFile.close();

    //Export coverage over time
    std::ofstream coverageFile;
    coverageFile.open(metric_path_str + "/coverage.csv");
    coverageFile << "tick,";
    for (auto & it : m_metrics.coverage_over_time) {
        coverageFile << it.first << ",";
    }
    coverageFile << "\n";

    //Get the size of the largest coverage vector
    int max_coverage_list_size = 0;
    for (auto & it : m_metrics.coverage_over_time) {
        max_coverage_list_size = std::max(max_coverage_list_size, int(it.second.size()));
    }

    for (int i = 0; i < max_coverage_list_size; i++) {
        coverageFile << (i+1)*coverage_update_tick_interval << ",";
        for (auto & it : m_metrics.coverage_over_time) {
            if (i < it.second.size())
                coverageFile << it.second[i] << ",";
            else coverageFile << ",";
        }
        coverageFile << "\n";
    }
    coverageFile.close();



    #ifdef USING_CONFIDENCE_TREE
    //Export certainty over time (average, free, occupied)
    std::ofstream certaintyFile;
    certaintyFile.open(metric_path_str + "/certainty.csv");
    certaintyFile << "tick,";
    for (auto & it : m_metrics.average_total_certainty_over_time) {
        certaintyFile << "all_" << it.first << ",";
    }
    for (auto & it : m_metrics.average_free_pheromone_over_time) {
        certaintyFile << "free_" << it.first << ",";
    }
    for (auto & it : m_metrics.average_occupied_pheromone_over_time) {
        certaintyFile << "occupied_" << it.first << ",";
    }
    certaintyFile << "\n";

    //Get the size of the largest certainty vector
    int max_certainty_list_size = 0;
    for (auto & it : m_metrics.average_total_certainty_over_time) {
        max_certainty_list_size = std::max(max_certainty_list_size, int(it.second.size()));
    }

    for (int i = 0; i < max_certainty_list_size; i++) {
        certaintyFile << (i+1)*coverage_update_tick_interval << ",";
        for (auto & it : m_metrics.average_total_certainty_over_time) {
            if (i < it.second.size())
                certaintyFile << it.second[i] << ",";
            else certaintyFile << ",";
        }
        for (auto & it : m_metrics.average_free_pheromone_over_time) {
            if (i < it.second.size())
                certaintyFile << it.second[i] << ",";
            else certaintyFile << ",";
        }
        for (auto & it : m_metrics.average_occupied_pheromone_over_time) {
            if (i < it.second.size())
                certaintyFile << it.second[i] << ",";
            else certaintyFile << ",";
        }
        certaintyFile << "\n";
    }
    certaintyFile.close();

    //Export certainty over time (average, free, occupied) area compensated
    std::ofstream certaintyAreaCompensatedFile;
    certaintyAreaCompensatedFile.open(metric_path_str + "/certainty_area_compensated.csv");
    certaintyAreaCompensatedFile << "tick,";
    for (auto & it : m_metrics.average_total_certainty_over_time_areacompensated) {
        certaintyAreaCompensatedFile << "all_" << it.first << ",";
    }
    for (auto & it : m_metrics.average_free_pheromone_over_time_areacompensated) {
        certaintyAreaCompensatedFile << "free_" << it.first << ",";
    }
    for (auto & it : m_metrics.average_occupied_pheromone_over_time_areacompensated) {
        certaintyAreaCompensatedFile << "occupied_" << it.first << ",";
    }
    certaintyAreaCompensatedFile << "\n";

    //Get the size of the largest certainty vector
    int max_certainty_area_compensated_list_size = 0;
    for (auto & it : m_metrics.average_total_certainty_over_time_areacompensated) {
        max_certainty_area_compensated_list_size = std::max(max_certainty_area_compensated_list_size, int(it.second.size()));
    }

    for (int i = 0; i < max_certainty_area_compensated_list_size; i++) {
        certaintyAreaCompensatedFile << (i+1)*coverage_update_tick_interval << ",";
        for (auto & it : m_metrics.average_total_certainty_over_time_areacompensated) {
            if (i < it.second.size())
                certaintyAreaCompensatedFile << it.second[i] << ",";
            else certaintyAreaCompensatedFile << ",";
        }
        for (auto & it : m_metrics.average_free_pheromone_over_time_areacompensated) {
            if (i < it.second.size())
                certaintyAreaCompensatedFile << it.second[i] << ",";
            else certaintyAreaCompensatedFile << ",";
        }
        for (auto & it : m_metrics.average_occupied_pheromone_over_time_areacompensated) {
            if (i < it.second.size())
                certaintyAreaCompensatedFile << it.second[i] << ",";
            else certaintyAreaCompensatedFile << ",";
        }
        certaintyAreaCompensatedFile << "\n";
    }
    certaintyAreaCompensatedFile.close();

    //Export number of cells and leaves
    std::ofstream numberOfCellsAndLeavesFile;
    numberOfCellsAndLeavesFile.open(metric_path_str + "/number_of_cells_and_leaves.csv");
    numberOfCellsAndLeavesFile << "tick,";
    for (auto & it : m_metrics.number_of_cells_and_leaves_over_time) {
        numberOfCellsAndLeavesFile << "cells_" << it.first << ",";
        numberOfCellsAndLeavesFile << "leaves_" << it.first << ",";
    }
    numberOfCellsAndLeavesFile << "\n";

    //Get the size of the largest number of cells and leaves vector
    int max_cells_and_leaves_list_size = 0;
    for (auto & it : m_metrics.number_of_cells_and_leaves_over_time) {
        max_cells_and_leaves_list_size = std::max(max_cells_and_leaves_list_size, int(it.second.size()));
    }

    for (int i = 0; i < max_cells_and_leaves_list_size; i++) {
        numberOfCellsAndLeavesFile << (i+1)*coverage_update_tick_interval << ",";
        for (auto & it : m_metrics.number_of_cells_and_leaves_over_time) {
            if (i < it.second.size())
                numberOfCellsAndLeavesFile << it.second[i].first << ",";
            else numberOfCellsAndLeavesFile << ",";
            if (i < it.second.size())
                numberOfCellsAndLeavesFile << it.second[i].second << ",";
            else numberOfCellsAndLeavesFile << ",";
        }
        numberOfCellsAndLeavesFile << "\n";
    }
    numberOfCellsAndLeavesFile.close();

   exportQuadtree("quadtree_all_done");
    #else
    exportMatrices("all_done");
    #endif



    //Export traveled path length
    std::ofstream traveledPathFile;
    traveledPathFile.open(metric_path_str + "/traveled_path.csv");
    traveledPathFile << "agent_id,traveled_path\n";
    for (auto & it : m_metrics.total_traveled_path) {
        traveledPathFile << it.first << "," << it.second << "\n";
    }
    traveledPathFile.close();

    //Export battery usage
    std::ofstream batteryUsageFile;
    batteryUsageFile.open(metric_path_str + "/battery_usage.csv");
    batteryUsageFile << "agent_id,battery_usage\n";
    for (auto & it : m_metrics.total_battery_usage) {
        batteryUsageFile << it.first << "," << it.second << "\n";
    }
    batteryUsageFile.close();

    //Export map observation count
    std::ofstream mapObservationCountFile;
    mapObservationCountFile.open(metric_path_str + "/map_observation_count.csv");
    mapObservationCountFile << "x,y,observation_count_total,";
    for (auto & it : m_metrics.map_observation_count) {
        mapObservationCountFile << it.first << ",";
    }
    mapObservationCountFile << "\n";
    for (int i = 0; i < m_metrics.map_observation_count_total.size(); i++) {
        for (int j = 0; j < m_metrics.map_observation_count_total[0].size(); j++) {
            mapObservationCountFile << i << ",";
            mapObservationCountFile << j << ",";
            mapObservationCountFile << m_metrics.map_observation_count_total[i][j] << ",";
            for (auto & it : m_metrics.map_observation_count) {
                mapObservationCountFile << m_metrics.map_observation_count[it.first][i][j] << ",";
            }
            mapObservationCountFile << "\n";
        }
    }
    mapObservationCountFile.close();

    //Export sent and received bytes
    std::ofstream bytesSentReceivedFile;
    bytesSentReceivedFile.open(metric_path_str + "/bytes_sent_received.csv");
    bytesSentReceivedFile << "agent_id,bytes_sent,bytes_received\n";
    for (auto & it : m_metrics.bytes_sent_received) {
        bytesSentReceivedFile << it.first << "," << it.second.first << "," << it.second.second << "\n";
    }
    bytesSentReceivedFile.close();



    //Export mission time
    std::ofstream missionTimeFile;
    missionTimeFile.open(metric_path_str + "/mission_time.csv");
    missionTimeFile << "agent_id,mission_time\n";
    for (auto & it : m_metrics.mission_time) {
        missionTimeFile << it.first << "," << it.second << "\n";
    }
    missionTimeFile.close();

}

#ifdef USING_CONFIDENCE_TREE
void CAgentVisionLoopFunctions::exportQuadtree(std::string filename) {
    //Export quadtree of each agent
    std::ofstream quadTreeFile;
    quadTreeFile.open(this->metric_path_str + "/" + filename + ".csv");
    quadTreeFile << "agent_id,box_x,box_y,box_size,pheromone\n";
    for (auto & it : m_tQuadTree) {
        for (auto & box: it.second) {
            quadTreeFile << it.first->GetId() << ",";
            quadTreeFile << std::get<0>(box).getCenter().x << ",";
            quadTreeFile << std::get<0>(box).getCenter().y << ",";
            quadTreeFile << std::get<0>(box).getSize() << ",";
            quadTreeFile << std::get<1>(box) << ",";
            quadTreeFile << "\n";
        }
    }
    quadTreeFile.close();
}

void CAgentVisionLoopFunctions::exportQuadtree(std::string filename, CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    pushQuadTree(pcFB, agent);
    //Export quadtree of each agent
    std::ofstream quadTreeFile;
    quadTreeFile.open(this->metric_path_str + "/" + filename + ".csv");
    quadTreeFile << "agent_id,box_x,box_y,box_size,pheromone\n";

    auto quadTree = m_tQuadTree[pcFB];

    for (auto & box: quadTree) {
        quadTreeFile << agent->id << ",";
        quadTreeFile << std::get<0>(box).getCenter().x << ",";
        quadTreeFile << std::get<0>(box).getCenter().y << ",";
        quadTreeFile << std::get<0>(box).getSize() << ",";
        quadTreeFile << std::get<1>(box) << ",";
        quadTreeFile << "\n";
    }
    quadTreeFile.close();
}
#else
void CAgentVisionLoopFunctions::exportMatrices(std::string filename) {
    //Export coverage matrix of each agent
    std::ofstream coverageMatrixFile;
    coverageMatrixFile.open(metric_path_str + "/coverage_matrix_"+filename+".csv");
    coverageMatrixFile << "agent_id,x,y,coverage_pheromone,size\n";
    for (auto & it : m_tCoverageMatrix) {
        for (int i = 0; i < it.second.size(); i++) {
            for (int j = 0; j < it.second[0].size(); j++) {
                if (it.second[i][j] == 0) continue; //Skip empty cells, only export visited cells to save space
//                if (it.second[i][j] == -1 && m_tObstacleMatrix[it.first][i][j] == -1) continue; //Skip empty cells
                coverageMatrixFile << it.first->GetId() << ",";
                auto realCoords = getRealCoordinateFromIndex(i, j, coverageMatrixResolution);
                coverageMatrixFile << realCoords.x<< ",";
                coverageMatrixFile << realCoords.y << ",";
                coverageMatrixFile << it.second[i][j];
                coverageMatrixFile << "," << coverageMatrixResolution << "\n";
            }
        }
    }
    coverageMatrixFile.close();

    //Export obstacle matrix of each agent
    std::ofstream obstacleMatrixFile;
    obstacleMatrixFile.open(metric_path_str + "/obstacle_matrix_"+filename+".csv");
    obstacleMatrixFile << "agent_id,x,y,obstacle_pheromone,size\n";
    for (auto & it : m_tObstacleMatrix) {
        for (int i = 0; i < it.second.size(); i++) {
            for (int j = 0; j < it.second[0].size(); j++) {
                if (it.second[i][j] == 0) continue; //Skip empty cells, only export visited cells to save space
//                if (it.second[i][j] == -1 && m_tCoverageMatrix[it.first][i][j] == -1) continue;
                obstacleMatrixFile << it.first->GetId() << ",";
                auto realCoords = getRealCoordinateFromIndex(i, j, obstacleMatrixResolution);
                obstacleMatrixFile << realCoords.x<< ",";
                obstacleMatrixFile << realCoords.y << ",";
                obstacleMatrixFile << it.second[i][j];
                obstacleMatrixFile << "," << obstacleMatrixResolution << "\n";
            }
        }
    }
    obstacleMatrixFile.close();

}


void CAgentVisionLoopFunctions::exportMatrices(std::string filename, CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    pushMatrices(pcFB, agent);

    auto coverageMatrixForExport = m_tCoverageMatrix[pcFB];
    auto obstacleMatrixForExport = m_tObstacleMatrix[pcFB];

    //Export coverage matrix of each agent
    std::ofstream coverageMatrixFile;
    coverageMatrixFile.open(metric_path_str + "/coverage_matrix_"+filename+".csv");
    coverageMatrixFile << "agent_id,x,y,coverage_pheromone,size\n";
    for (int i = 0; i < coverageMatrixForExport.size(); i++) {
        for (int j = 0; j < coverageMatrixForExport[0].size(); j++) {
//            if (coverageMatrixForExport[i][j] == -1 && obstacleMatrixForExport[i][j] == -1) continue;
            if (coverageMatrixForExport[i][j] == 0) continue; //Skip empty cells, only export visited cells to save space
            coverageMatrixFile << agent->id << ",";
            auto realCoords = getRealCoordinateFromIndex(i, j, coverageMatrixResolution);
            coverageMatrixFile << realCoords.x<< ",";
            coverageMatrixFile << realCoords.y << ",";
            coverageMatrixFile << coverageMatrixForExport[i][j];
            coverageMatrixFile << "," << coverageMatrixResolution << "\n";
        }
    }
    coverageMatrixFile.close();

    //Export obstacle matrix of each agent
    std::ofstream obstacleMatrixFile;
    obstacleMatrixFile.open(metric_path_str + "/obstacle_matrix_"+filename+".csv");
    obstacleMatrixFile << "agent_id,x,y,obstacle_pheromone,size\n";
    for (int i = 0; i < obstacleMatrixForExport.size(); i++) {
        for (int j = 0; j < obstacleMatrixForExport[0].size(); j++) {
//            if (obstacleMatrixForExport[i][j] == -1 && coverageMatrixForExport[i][j] == -1) continue;
            if (obstacleMatrixForExport[i][j] == 0) continue; //Skip empty cells, only export visited cells to save space
            obstacleMatrixFile << agent->id << ",";
            auto realCoords = getRealCoordinateFromIndex(i, j, obstacleMatrixResolution);
            obstacleMatrixFile << realCoords.x<< ",";
            obstacleMatrixFile << realCoords.y << ",";
            obstacleMatrixFile << obstacleMatrixForExport[i][j];
            obstacleMatrixFile << "," << obstacleMatrixResolution << "\n";
        }
    }
    obstacleMatrixFile.close();
}
#endif

/**
 * Check if a new agent has finished its mission
 * @param tFBMap
 * @return
 */
bool CAgentVisionLoopFunctions::allAgentsDone(CSpace::TMapPerType &tFBMap){
    bool allAgentsDone = true;
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        if (agent->state != Agent::State::MAP_RELAYED){
            allAgentsDone = false;
            if (agent->state == Agent::State::RETURNING){
                if (std::find(this->agents_returning.begin(), agents_returning.end(), pcFB->GetId()) == agents_returning.end()){
                    #ifdef USING_CONFIDENCE_TREE
                    //Export quadtree at point of return
                    exportQuadtree("quadtree_returning_" + pcFB->GetId(), pcFB, agent);
                    #else
                    exportMatrices("returning_" + pcFB->GetId(), pcFB, agent);
                    #endif
                    agents_returning.push_back(pcFB->GetId());
                }
            } else if (agent->state == Agent::State::FINISHED_EXPLORING){
                if (std::find(this->agents_finished_exploring.begin(), agents_finished_exploring.end(), pcFB->GetId()) == agents_finished_exploring.end()){
                    #ifdef USING_CONFIDENCE_TREE
                    //Export quadtree at point of return
                    exportQuadtree("quadtree_finished_exploring_" + pcFB->GetId(), pcFB, agent);
                    #else
                    exportMatrices("finished_exploring_" + pcFB->GetId(), pcFB, agent);
                    #endif
                    agents_finished_exploring.push_back(pcFB->GetId());
                }
            }
        } else {
            //If new agent done
            if (std::find(this->agents_relayed_map.begin(), agents_relayed_map.end(), pcFB->GetId()) == agents_relayed_map.end()){
                #ifdef USING_CONFIDENCE_TREE
                exportQuadtree("quadtree_map_relayed_" + pcFB->GetId(), pcFB, agent);
                #else
                exportMatrices("map_relayed_" + pcFB->GetId(), pcFB, agent);
                #endif
                agents_relayed_map.push_back(pcFB->GetId());
                m_metrics.mission_time[pcFB->GetId()] = agent->elapsed_ticks / agent->ticks_per_second;
            }
        }
    }
    return allAgentsDone;
}



void CAgentVisionLoopFunctions::updateBatteryUsage(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    double batteryUsage = agent->batteryManager.battery.getStateOfCharge();
    m_metrics.total_battery_usage[pcFB->GetId()] = (1.0-batteryUsage)*agent->config.BATTERY_CAPACITY; //In mAh
}

void CAgentVisionLoopFunctions::updateCellObservationCount(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    //Only update if agent is in mission or returning from mission
    if (agent->state == Agent::State::NO_MISSION || agent->state == Agent::State::FINISHED_EXPLORING || agent->state == Agent::State::MAP_RELAYED) return;
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

    Coordinate agentRealPosition = cController.getActualAgentPosition();
    argos::CRadians agentRealHeading = cController.getActualAgentOrientation();
//    bool addedObjectAtAgentLocation = false;
//    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
//        argos::CRadians sensor_rotation = agent->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
//        if (agent->distance_sensors[sensor_index].getDistance() < agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {
//
//            double opposite = argos::Sin(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();
//            double adjacent = argos::Cos(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();
//
//
//            Coordinate object = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
//            //If the detected object is actually another agent, add it as a free area
//            //So check if the object coordinate is close to another agent
//            bool close_to_other_agent = false;
//            for (const auto &agentLocation: agent->agentLocations) {
//                if ((agent->getTimeFromAgentLocation(agentLocation.first) - agent->elapsed_ticks) / agent->ticks_per_second >
//                    agent->config.AGENT_LOCATION_RELEVANT_S)
//                    continue;
//                argos::CVector2 objectToAgent =
//                        argos::CVector2(std::get<0>(agentLocation.second).x, std::get<0>(agentLocation.second).y)
//                        - argos::CVector2(object.x, object.y);
//
//                //If detected object and another agent are not close, add the object as an obstacle
//                if (objectToAgent.Length() <= agent->quadtree->getResolution()) {
//                    close_to_other_agent = true; //TODO: Due to confidence, can maybe omit this check
//                }
//            }
//            //Only add the object as an obstacle if it is not close to another agent
//            if (!close_to_other_agent) {
//                if (sqrt(pow(agentRealPosition.x - object.x, 2) + pow(agentRealPosition.y - object.y, 2)) <
//                    agent->quadtree->getResolution()) {
//                    addedObjectAtAgentLocation = true;
//                }
////                quadtree::Box objectBox = addObjectLocation(object, sensor_probability);
//                std::pair<int, int> mapIndex = coordinateToMapIndex(object, agent);
//                m_metrics.map_observation_count_total[mapIndex.first][mapIndex.second]++;
//
//                if (!addedObjectAtAgentLocation) {
//                    observeAreaBetween(agentRealPosition, object, agent);
//                }
//            } else {
//                if (!addedObjectAtAgentLocation) observeAreaBetween(agentRealPosition, object, agent);
//            }
//
//
//        } else {
//            double opposite = argos::Sin(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
//            double adjacent = argos::Cos(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
//
//
//            Coordinate end_of_ray = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
//            if (!addedObjectAtAgentLocation) observeAreaBetween(agentRealPosition, end_of_ray, agent);
//        }
//    }
    bool addedObjectAtAgentLocation = false;
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = agentRealHeading - sensor_index * argos::CRadians::PI_OVER_TWO;
        if (agent->distance_sensors[sensor_index].getDistance() < agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {

            double opposite = argos::Sin(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();
            double adjacent = argos::Cos(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();

            Coordinate object = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
//            std::pair<int, int> mapIndex = coordinateToMapIndex(object, agent);
//            m_metrics.map_observation_count_total[mapIndex.first][mapIndex.second]++;
            observeAreaBetween(agentRealPosition, object, agent);

        } else {
            double opposite = argos::Sin(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;

            Coordinate end_of_ray = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
            observeAreaBetween(agentRealPosition, end_of_ray, agent);
        }
    }
}


std::pair<int, int> CAgentVisionLoopFunctions::coordinateToMapIndex(Coordinate coordinate, const std::shared_ptr<Agent> &agent) {
    #ifdef USING_CONFIDENCE_TREE
    double rootboxSize = agent->quadtree->getRootBox().getSize();
    #else
    double rootboxSize = agent->coverageMatrix->getRealWidth();
    #endif
    int x = std::floor((coordinate.x + rootboxSize/2.0) / rootboxSize * this->m_metrics.map_observation_count_total.size());
    int y = std::floor((coordinate.y + rootboxSize/2.0) / rootboxSize * this->m_metrics.map_observation_count_total[0].size());
    return std::make_pair(x, y);
}

void CAgentVisionLoopFunctions::observeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, const std::shared_ptr<Agent> &agent) {
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = agent->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
//        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//        this->m_metrics.map_observation_count_total[coordinateToMapIndex(Coordinate{x + 0.0000000001, y + 0.0000000001}, agent).first][coordinateToMapIndex(Coordinate{x + 0.0000000001, y + 0.0000000001}, agent).second]++;
//        x += stepX;
//        y += stepY;
//    }

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(agent.get(), coordinate1,
                                                                                    coordinate2);
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        std::pair<int, int> mapIndexCoordinate = coordinateToMapIndex(Coordinate{point.x, point.y}, agent);
        if (mapIndexCoordinate.first >= 0 && mapIndexCoordinate.first < this->m_metrics.map_observation_count_total.size() &&
            mapIndexCoordinate.second >= 0 && mapIndexCoordinate.second < this->m_metrics.map_observation_count_total[0].size()) {
            this->m_metrics.map_observation_count_total[mapIndexCoordinate.first][mapIndexCoordinate.second]++;
            this->m_metrics.map_observation_count[agent->id][mapIndexCoordinate.first][mapIndexCoordinate.second]++;
        }
        else assert(0);
    }
}

void CAgentVisionLoopFunctions::checkReturnToDeploymentSite(CSpace::TMapPerType &tFBMap) {
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        Coordinate agentRealPosition = cController.getActualAgentPosition();
        //Check if agent is within 1m from deployment position
        auto distance = sqrt(pow(agentRealPosition.x - this->deployment_positions[pcFB->GetId()].x, 2) +
                             pow(agentRealPosition.y - this->deployment_positions[pcFB->GetId()].y, 2));
        this->m_metrics.distance_to_deployment_site[pcFB->GetId()] = distance;
    }
}

bool CAgentVisionLoopFunctions::IsExperimentFinished() {
    return experimentFinished;
}

void CAgentVisionLoopFunctions::PostExperiment() {
    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end-start;

    argos::LOG << "Experiment time: " << (elapsed_seconds.count()) << "s : " << elapsed_seconds.count()/60.0 << "min" << std::endl;
    argos::LOG.Flush();
    exit(0);
}

void CAgentVisionLoopFunctions::updateBytesSentReceived(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    std::vector<std::string> messages = {};
    agent->wifi.checkMessagesInTransitPeek(messages, agent->elapsed_ticks/agent->ticks_per_second);
    auto receivedBytes = 0;
    for (const auto &message: messages) {
        receivedBytes += message.size();
    }
    m_metrics.bytes_sent_received[pcFB->GetId()].second += receivedBytes;
    auto sentMessages = agent->wifi.radioActuator->GetInterfaces()[0].Messages;
    auto sentBytes = 0;
    for (const auto &message: sentMessages) {
        sentBytes += message.Size();
    }
    m_metrics.bytes_sent_received[pcFB->GetId()].first += sentBytes;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAgentVisionLoopFunctions, "agent_vision_loop_functions_pipuck")
