//
// Created by hugo on 28-11-24.
//

#include "PathFollower.h"
#include "agent_implementation/agent.h"

void PathFollower::followPath(Agent *agent,
                              std::set<argos::CDegrees, CustomComparator> &freeAngles,
                              argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {
    if (agent->route_to_best_frontier.empty()) return;

//    for (auto [start, end]: agent->route_to_best_frontier) {
//        argos::LOG << "Route: " << start.x << ", " << start.y << " to " << end.x << ", " << end.y << std::endl;
//    }
//    for (auto [start, end]: current_route){
//        argos::LOG << "Current route: " << start.x << ", " << start.y << " to " << end.x << ", " << end.y << std::endl;
//    }
    if (current_route != agent->route_to_best_frontier) {
        current_path_section = 0;
    }

    current_route = agent->route_to_best_frontier;

    if (current_path_section >= agent->route_to_best_frontier.size()) {
        return;
    }


    argos::LOG << "route size: " << agent->route_to_best_frontier.size() << " and current path section: " << current_path_section << std::endl;
    auto [start, end] = agent->route_to_best_frontier.at(current_path_section);

    //Our target is the end of the edge
    Coordinate sub_target = end;

    //If we are close to the target, go to the next section
    if (sqrt(pow(sub_target.x - agent->position.x, 2) + pow(sub_target.y - agent->position.y, 2)) < agent->OBJECT_AVOIDANCE_RADIUS*2) {
        current_path_section++;
        //If we are at the end of the path, return
        if (current_path_section >= agent->route_to_best_frontier.size()) {
            return;
        }
    }

    //Calculate the angle to the target
    argos::CRadians angleToSubTarget = argos::ATan2(sub_target.y - agent->position.y, sub_target.x - agent->position.x);

    //Get the free angle closest to the target angle
    auto closestFreeAngle = *freeAngles.begin();
    for (auto freeAngle: freeAngles) {
        if (std::abs(NormalizedDifference(ToRadians(freeAngle), angleToSubTarget).GetValue()) <
            std::abs(NormalizedDifference(ToRadians(closestFreeAngle), angleToSubTarget).GetValue())) {
            closestFreeAngle = freeAngle;
        }
    }

    //Calculate the relative object avoidance angle
    *relativeObjectAvoidanceAngle = NormalizedDifference(ToRadians(closestFreeAngle), targetAngle);

}
