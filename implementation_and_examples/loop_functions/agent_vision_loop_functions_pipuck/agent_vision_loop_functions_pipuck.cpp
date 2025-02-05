#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_model.h> // Include the correct header
#include <set>
#include "agent_vision_loop_functions_pipuck.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include <chrono>
#include <sys/stat.h>
#include <cstdlib>  // for getenv()

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
                                                                                   agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE * 2.0);

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
        Coordinate agentLocation = (std::get<0>(it.second));
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
    std::vector<std::tuple<quadtree::Box, double>> boxesAndPheromones = agent->quadtree->getAllBoxes(globalElapsedTime);
    m_tNeighborPairs[pcFB] = agent->quadtree->getAllNeighborPairs();

    m_tQuadTree[pcFB] = boxesAndPheromones;
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
    int map_cols = 0;
    int map_rows = 0;
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;

        agent->ticks_per_second = ticksPerSecond;

        currently_colliding[pcFB] = false;
        previous_positions[pcFB] = Coordinate{MAXFLOAT, MAXFLOAT};
        map_cols = std::max(map_cols, int(agent->quadtree->getRootBox().getSize()/agent->quadtree->getResolution()));
        map_rows = std::max(map_rows, int(agent->quadtree->getRootBox().getSize()/agent->quadtree->getResolution()));

        Coordinate agentRealPosition = cController.getActualAgentPosition();
        deployment_positions[pcFB->GetId()] = agentRealPosition;

        longest_mission_time_s = std::max(longest_mission_time_s, agent->config.MISSION_END_TIME_S);
    }

    this->m_metrics = metrics();
    this->m_metrics.map_observation_count.resize(map_cols, std::vector<int>(map_rows, 0));
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
        globalElapsedTime = agent->elapsed_ticks / agent->ticks_per_second;

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
        m_tAgentHeadings[pcFB] = Coordinate::OwnHeadingToArgos(agent->heading);
        pushQuadTree(pcFB, agent);

//        m_tAgentFrontiers[pcFB] = agent->current_frontiers;
        m_tAgentFrontierRegions[pcFB] = agent->current_frontier_regions;

//        m_tAgentFreeAngles[pcFB] = agent->freeAngles;
        for(auto angle: agent->freeAnglesVisualization) {
            m_tAgentFreeAngles[pcFB].insert(ToDegrees(Coordinate::OwnHeadingToArgos(ToRadians(angle))));
        }

#ifdef BATTERY_MANAGEMENT_ENABLED
        m_tAgentBatteryLevels[pcFB] = agent->batteryManager.battery.getStateOfCharge() * 100.0f;
#endif


        m_tAgentRoute[pcFB] = agent->route_to_best_frontier;

        updateCollisions(pcFB);
        updateTraveledPathLength(pcFB, agent);
        updateBatteryUsage(pcFB, agent);
        updateCellObservationCount(pcFB, agent);
    }

    auto mBox = quadtree::Box(-5.5, 5.5, 11);
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
        updateCoverage(it.first, it.second);
        updateCertainty(it.first, it.second);
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

    //If a new agent is done, update the metrics and maps
    if (allAgentsDone(tFBMap)) {
        updateAgentsFinishedTime(tFBMap);
        checkReturnToDeploymentSite(tFBMap);
        exportMetricsAndMaps();
    }

    loop_function_steps++;
}

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

void CAgentVisionLoopFunctions::updateCoverage(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double >>& tree) {
    //Get controller
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
    auto inMission = cController.agentObject->state != Agent::State::NO_MISSION && cController.agentObject->state != Agent::State::FINISHED;
    //Update coverage over time at every interval, if mission has started
    if (inMission && cController.agentObject->elapsed_ticks % coverage_update_tick_interval == 0){

        double covered_area = 0;


        //TODO: Consider irreachible area
        for (auto &it: tree) {
            quadtree::Box box = std::get<0>(it);

            double box_size = box.getSize();
            covered_area += box_size * box_size;
        }

        double coverage = covered_area / (cController.map_width * cController.map_height);
        argos::LOG << "[" << pcFB->GetId() << "] Coverage: " << coverage << std::endl;


        m_metrics.coverage_over_time[pcFB->GetId()].push_back(coverage);
    }
}

void CAgentVisionLoopFunctions::updateCertainty(argos::CPiPuckEntity *pcFB, const std::vector<std::tuple<quadtree::Box, double>> &tree) {
    //Get controller
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
    auto inMission = cController.agentObject->state != Agent::State::NO_MISSION && cController.agentObject->state != Agent::State::FINISHED;

        //Update certainty over time at every interval, if mission has started
        if (inMission && cController.agentObject->elapsed_ticks % coverage_update_tick_interval == 0){
        double total_certainty = 0;
        int total_boxes = 0;
        double free_certainty = 0;
        int free_boxes = 0;
        double occupied_certainty = 0;
        int occupied_boxes = 0;


        for (auto & it : tree) {
            auto pheromone = std::get<1>(it);
            total_certainty += std::abs(pheromone-0.5);
            total_boxes++;
            if (pheromone > 0.5) {
                free_certainty += pheromone;
                free_boxes++;
            } else if (pheromone <= 0.5) {
                occupied_certainty += pheromone;
                occupied_boxes++;
            }
        }

        double average_total_certainty = total_certainty / total_boxes;
        double average_free_certainty = free_certainty / free_boxes;
        double average_occupied_certainty = occupied_certainty / occupied_boxes;
        argos::LOG << "[" << pcFB->GetId() << "] Average total certainty: " << average_total_certainty << std::endl;
        argos::LOG << "[" << pcFB->GetId() << "] Average free certainty: " << average_free_certainty << std::endl;
        argos::LOG << "[" << pcFB->GetId() << "] Average occupied certainty: " << average_occupied_certainty << std::endl;


        m_metrics.average_total_certainty_over_time[pcFB->GetId()].push_back(average_total_certainty);
        m_metrics.average_free_pheromone_over_time[pcFB->GetId()].push_back(average_free_certainty);
        m_metrics.average_occupied_pheromone_over_time[pcFB->GetId()].push_back(average_occupied_certainty);
    }

}

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

    std::string experiment_name_str = "experiment";
    const char* experiment_name = std::getenv("EXPERIMENT");
    if (experiment_name) {
        //Remove ".argos"
        experiment_name_str = experiment_name;
        experiment_name_str = experiment_name_str.substr(0, experiment_name_str.find_last_of('.'));
    }

    if (mkdir(("experiment_results/" + experiment_name_str).c_str(), 0777) == -1) {
        std::cerr << "Error :  " << strerror(errno) << std::endl;
    }


    //Export metrics
    std::ofstream metricsFile;
    metricsFile.open("experiment_results/" + experiment_name_str + "/metrics.csv");
    metricsFile << "n_agent_agent_collisions,";
    metricsFile << "n_agent_obstacle_collisions\n";
    metricsFile << m_metrics.n_agent_agent_collisions << ",";
    metricsFile << m_metrics.n_agent_obstacle_collisions << "\n";
    metricsFile.close();

    //Export agent returned to deployment site
    std::ofstream returnedToDeploymentSiteFile;
    returnedToDeploymentSiteFile.open("experiment_results/" + experiment_name_str + "/returned_to_deployment_site.csv");
    returnedToDeploymentSiteFile << "agent_id,returned_to_deployment_site\n";
    for (auto & it : m_metrics.returned_to_deployment_site) {
        returnedToDeploymentSiteFile << it.first << "," << it.second << "\n";
    }
    returnedToDeploymentSiteFile.close();

    //Export coverage over time
    std::ofstream coverageFile;
    coverageFile.open("experiment_results/" + experiment_name_str + "/coverage.csv");
    coverageFile << "time_s,";
    for (auto & it : m_metrics.coverage_over_time) {
        coverageFile << it.first << ",";
    }
    coverageFile << "\n";
    for (int i = 0; i < m_metrics.coverage_over_time.begin()->second.size(); i++) {
        coverageFile << i*coverage_update_tick_interval << ",";
        for (auto & it : m_metrics.coverage_over_time) {
            if (i < it.second.size())
                coverageFile << it.second[i] << ",";
            else coverageFile << ",";
        }
        coverageFile << "\n";
    }
    coverageFile.close();

    //Export traveled path length
    std::ofstream traveledPathFile;
    traveledPathFile.open("experiment_results/" + experiment_name_str + "/traveled_path.csv");
    traveledPathFile << "agent_id,traveled_path\n";
    for (auto & it : m_metrics.total_traveled_path) {
        traveledPathFile << it.first << "," << it.second << "\n";
    }
    traveledPathFile.close();

    //Export battery usage
    std::ofstream batteryUsageFile;
    batteryUsageFile.open("experiment_results/" + experiment_name_str + "/battery_usage.csv");
    batteryUsageFile << "agent_id,battery_usage\n";
    for (auto & it : m_metrics.total_battery_usage) {
        batteryUsageFile << it.first << "," << it.second << "\n";
    }
    batteryUsageFile.close();

    //Export quadtree of each agent
    std::ofstream quadTreeFile;
    quadTreeFile.open("experiment_results/" + experiment_name_str + "/quadtree.csv");
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

    //Export map observation count
    std::ofstream mapObservationCountFile;
    mapObservationCountFile.open("experiment_results/" + experiment_name_str + "/map_observation_count.csv");
    mapObservationCountFile << "x,y,observation_count\n";
    for (int i = 0; i < m_metrics.map_observation_count.size(); i++) {
        for (int j = 0; j < m_metrics.map_observation_count[0].size(); j++) {
            mapObservationCountFile << i << ",";
            mapObservationCountFile << j << ",";
            mapObservationCountFile << m_metrics.map_observation_count[i][j] << "\n";
        }
    }
    mapObservationCountFile.close();

    //Export mission time
    std::ofstream missionTimeFile;
    missionTimeFile.open("experiment_results/" + experiment_name_str + "/mission_time.csv");
    missionTimeFile << "agent_id,mission_time\n";
    for (auto & it : m_metrics.mission_time) {
        missionTimeFile << it.first << "," << it.second << "\n";
    }
    missionTimeFile.close();

}

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

        if (agent->state != Agent::State::FINISHED){
            allAgentsDone = false;
        }
    }
    experimentFinished = allAgentsDone; //If all agents are done, the experiment is finished
    return experimentFinished;
}


void CAgentVisionLoopFunctions::updateAgentsFinishedTime(CSpace::TMapPerType &tFBMap) {
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        std::shared_ptr<Agent> agent = cController.agentObject;
        if (agent->state == Agent::State::FINISHED){
            //If no value yet
            if (m_metrics.mission_time.find(pcFB->GetId()) == m_metrics.mission_time.end()){
                m_metrics.mission_time[pcFB->GetId()] = agent->elapsed_ticks / agent->ticks_per_second;
            }
        }

    }

}


void CAgentVisionLoopFunctions::updateBatteryUsage(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    double batteryUsage = agent->batteryManager.battery.getStateOfCharge();
    m_metrics.total_battery_usage[pcFB->GetId()] = (1.0-batteryUsage)*100.0;
}

void CAgentVisionLoopFunctions::updateCellObservationCount(CPiPuckEntity *pcFB, const std::shared_ptr<Agent> &agent) {
    //Only update if agent is in mission or returning from mission
    if (agent->state == Agent::State::NO_MISSION || agent->state == Agent::State::FINISHED) return;
    auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());

    Coordinate agentRealPosition = cController.getActualAgentPosition();
    bool addedObjectAtAgentLocation = false;
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = agent->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        if (agent->distance_sensors[sensor_index].getDistance() < agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {

            float sensor_probability = HC_SR04::getProbability(agent->distance_sensors[sensor_index].getDistance());

            double opposite = argos::Sin(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();
            double adjacent = argos::Cos(sensor_rotation) * agent->distance_sensors[sensor_index].getDistance();


            Coordinate object = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
            bool close_to_other_agent = false;
            for (const auto &agentLocation: agent->agentLocations) {
                if ((std::get<2>(agentLocation.second) - agent->elapsed_ticks) / agent->ticks_per_second >
                        agent->config.AGENT_LOCATION_RELEVANT_S)
                    continue;
                argos::CVector2 objectToAgent =
                        argos::CVector2(std::get<0>(agentLocation.second).x, std::get<0>(agentLocation.second).y)
                        - argos::CVector2(object.x, object.y);

                //If detected object and another agent are not close, add the object as an obstacle
                if (objectToAgent.Length() <= agent->quadtree->getResolution()) {
                    close_to_other_agent = true; //TODO: Due to confidence, can maybe omit this check
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if (!close_to_other_agent) {
                if (sqrt(pow(agentRealPosition.x - object.x, 2) + pow(agentRealPosition.y - object.y, 2)) <
                        agent->quadtree->getResolution()) {
                    addedObjectAtAgentLocation = true;
                }
//                quadtree::Box objectBox = addObjectLocation(object, sensor_probability);
                std::pair<int, int> mapIndex = coordinateToMapIndex(object, agent);
                m_metrics.map_observation_count[mapIndex.first][mapIndex.second]++;

                if (!addedObjectAtAgentLocation) {
                    observeAreaBetween(agentRealPosition, object, agent);
                }
            } else {
                if (!addedObjectAtAgentLocation) observeAreaBetween(agentRealPosition, object, agent);
            }


        } else {
            double opposite = argos::Sin(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * agent->config.DISTANCE_SENSOR_PROXIMITY_RANGE;


            Coordinate end_of_ray = {agentRealPosition.x + adjacent, agentRealPosition.y + opposite};
            if (!addedObjectAtAgentLocation) observeAreaBetween(agentRealPosition, end_of_ray, agent);
        }
    }

}

std::pair<int, int> CAgentVisionLoopFunctions::coordinateToMapIndex(Coordinate coordinate, const std::shared_ptr<Agent> &agent) {
    double rootboxSize = agent->quadtree->getRootBox().getSize();
    int x = std::floor((coordinate.x + rootboxSize/2.0) / rootboxSize * this->m_metrics.map_observation_count.size());
    int y = std::floor((coordinate.y + rootboxSize/2.0) / rootboxSize * this->m_metrics.map_observation_count[0].size());
    return std::make_pair(x, y);
}

void CAgentVisionLoopFunctions::observeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, const std::shared_ptr<Agent> &agent) {
    double x = coordinate1.x;
    double y = coordinate1.y;
    double dx = coordinate2.x - coordinate1.x;
    double dy = coordinate2.y - coordinate1.y;
    double distance = sqrt(dx * dx + dy * dy);
    double stepSize = agent->quadtree->getResolution();
    int nSteps = std::ceil(distance / stepSize);
    double stepX = dx / nSteps;
    double stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
        this->m_metrics.map_observation_count[coordinateToMapIndex(Coordinate{x + 0.0000000001, y + 0.0000000001}, agent).first][coordinateToMapIndex(Coordinate{x + 0.0000000001, y + 0.0000000001}, agent).second]++;
        x += stepX;
        y += stepY;
    }
}

void CAgentVisionLoopFunctions::checkReturnToDeploymentSite(CSpace::TMapPerType &tFBMap) {
    for (auto & it : tFBMap) {
        /* Create a pointer to the current pi-puck */
        CPiPuckEntity *pcFB = any_cast<CPiPuckEntity *>(it.second);
        auto &cController = dynamic_cast<PiPuckHugo &>(pcFB->GetControllableEntity().GetController());
        Coordinate agentRealPosition = cController.getActualAgentPosition();
        //Check if agent is within 1m from deployment position
        if (sqrt(pow(agentRealPosition.x - this->deployment_positions[pcFB->GetId()].x, 2) +
                 pow(agentRealPosition.y - this->deployment_positions[pcFB->GetId()].y, 2)) <= 1.0) {
            this->m_metrics.returned_to_deployment_site[pcFB->GetId()] = true;
        } else {
            this->m_metrics.returned_to_deployment_site[pcFB->GetId()] = false;
        }
    }
}

bool CAgentVisionLoopFunctions::IsExperimentFinished() {
    return experimentFinished;
}

void CAgentVisionLoopFunctions::PostExperiment() {
    exit(0);
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAgentVisionLoopFunctions, "agent_vision_loop_functions_pipuck")
