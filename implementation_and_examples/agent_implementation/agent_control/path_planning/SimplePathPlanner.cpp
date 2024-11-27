
#include "SimplePathPlanner.h"
#include "agent_implementation/utils/Box.h"
#include "agent_implementation/agent.h"

///**
// * Get the route from start to end
// * Each section is relative to the former
// * @param agent
// * @param start
// * @param target
// * @param quadtree
// * @param closestFreeAngleRadians
// * @param relativeObjectAvoidanceAngle
// * @return
// */
//std::vector<argos::CVector2> SimplePathPlanner::getRoute(Agent* agent, Coordinate start, Coordinate target, const quadtree::Quadtree& quadtree, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle) const {
//    std::vector<argos::CVector2> relativeSections;
//    std::vector<std::pair<Coordinate, Coordinate>> route = getRouteSections(agent, start, target, quadtree, closestFreeAngleRadians, relativeObjectAvoidanceAngle);
//
//    //Compare first section with agent heading
//    auto [start_edge, end_edge] = route.at(0);
//    auto vector_in_coordinate_system = argos::CVector2(end_edge.x - start_edge.x, end_edge.y - start_edge.y);
//    auto relative_vector = vector_in_coordinate_system.Rotate(-agent->heading);
//    relativeSections.push_back(relative_vector);
//
//    for (int i=0; i<route.size()-1; i++) {
//        auto [start_edge1, end_edge1] = route.at(i);
//        auto vector_in_coordinate_system_1 = argos::CVector2(end_edge1.x - start_edge1.x, end_edge1.y - start_edge1.y);
//        auto [start_edge2, end_edge2] = route.at(i+1);
//        auto vector_in_coordinate_system_2 = argos::CVector2(end_edge2.x - start_edge2.x, end_edge2.y - start_edge2.y);
//        auto relative_vector =
//
//    }
//    return relativeSections;
//}

std::vector<std::pair<Coordinate, Coordinate>> SimplePathPlanner::getRoute(Agent* agent, Coordinate start, Coordinate target, const quadtree::Quadtree& quadtree, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle) const {
    return getRouteSections(agent, start, target, quadtree, closestFreeAngleRadians, relativeObjectAvoidanceAngle);
}

std::vector<std::pair<Coordinate, Coordinate>> SimplePathPlanner::getRouteSections(Agent* agent, Coordinate start, Coordinate target, const quadtree::Quadtree& quadtree, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle) const {
    auto [cell, box, edge_index] = rayTraceQuadtreeIntersection(start, target, quadtree);
    if (cell == nullptr) return {};
    std::vector<std::pair<Coordinate, Coordinate>> route;
    auto [start_edge, end_edge] = getEdgeCoordinates(box, edge_index);
    route.push_back({start_edge, end_edge});

    getWallFollowingRoute(agent, cell, box.size, edge_index, target, quadtree, route, closestFreeAngleRadians, relativeObjectAvoidanceAngle);

    //Append last point to target
    route.push_back({route.rbegin()->second, target});

    return route;
}

void SimplePathPlanner::getWallFollowingRoute(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, const quadtree::Quadtree &quadtree, std::vector<std::pair<Coordinate, Coordinate>> & route, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle) const {
    if (std::abs(ToDegrees(*relativeObjectAvoidanceAngle).GetValue()) < agent->TURN_THRESHOLD_DEGREES) { //Direction to target free
        return;
    }
    auto edges = elegible_edges(cell, edge_index);

    quadtree::Quadtree::Cell * bestCell = nullptr;
    int bestEdgeIndex = -1;
    Coordinate bestEdgeStart = {MAXFLOAT, MAXFLOAT};
    Coordinate bestEdgeEnd = {MAXFLOAT, MAXFLOAT};

    double max_distance = -MAXFLOAT;

    for (auto [c, e]: edges) {
        auto cBoxTopLeft = Coordinate{c->quadNode.coordinate.x - box_size /2, c->quadNode.coordinate.y + box_size / 2};
        auto cBox = quadtree::Box(cBoxTopLeft, box_size);
        auto [start_edge, end_edge] = getEdgeCoordinates(cBox, e);
        auto edge_middle = Coordinate{(start_edge.x + end_edge.x) / 2, (start_edge.y + end_edge.y) / 2};

        auto cell_to_edge_distance = sqrt(pow(edge_middle.x - target.x, 2) + pow(edge_middle.y - target.y, 2));
        if (std::find(route.begin(), route.end(), std::pair{start_edge, end_edge}) == route.end() && cell_to_edge_distance > max_distance) {
            bestCell = c;
            bestEdgeIndex = e;
            bestEdgeStart = start_edge;
            bestEdgeEnd = end_edge;
            max_distance = cell_to_edge_distance;
        }
    }

    if (bestCell != nullptr) {
        route.push_back({bestEdgeStart, bestEdgeEnd});
        getWallFollowingRoute(agent, bestCell, box_size, bestEdgeIndex, target, quadtree, route, closestFreeAngleRadians, relativeObjectAvoidanceAngle);
    }

}

std::vector<std::pair<quadtree::Quadtree::Cell*, int>> SimplePathPlanner::elegible_edges(quadtree::Quadtree::Cell * cell, int edge_index) const{
    std::vector<std::pair<quadtree::Quadtree::Cell*, int>> edges;
    int opposite_edge_index = edge_index < 2 ? edge_index + 2 : edge_index - 2;

    edges.push_back({cell, (edge_index+1)%4});
    edges.push_back({cell, (edge_index+3)%4});

    for (int i = 0; i < 4; i++) {
        if (cell->neighbors.at(i) != nullptr) {
            auto index = std::find(edges.begin(), edges.end(), std::pair{cell, i});
            if (index != edges.end()) {
                edges.erase(index);
            }
            if (i != edge_index && i != opposite_edge_index) {
                if (cell->neighbors.at(i) != nullptr) {
                    if (cell->neighbors.at(i)->neighbors.at(edge_index) != nullptr) {
                        auto opposite_i = i < 2 ? i + 2 : i - 2;
                        edges.push_back({cell->neighbors.at(i)->neighbors.at(edge_index), opposite_i});
                    } else {
                        edges.push_back({cell->neighbors.at(i), edge_index});
                    }
                }
            }
        }
    }
}

std::tuple<quadtree::Quadtree::Cell*, quadtree::Box, int> SimplePathPlanner::rayTraceQuadtreeIntersection(Coordinate start, Coordinate target, const quadtree::Quadtree& quadtree) const {
    auto x = start.x;
    auto y = start.y;
    auto dx = target.x - start.x;
    auto dy = target.y - start.y;
    auto distance = sqrt(dx * dx + dy * dy);
    auto stepSize = 0.05;
    auto nSteps = std::ceil(distance / stepSize);
    auto stepX = dx / nSteps;
    auto stepY = dy / nSteps;

    for (int s = 0; s < nSteps; s++) {
        auto cell_and_box = quadtree.getCellandBoxFromCoordinate(Coordinate{x, y});
        auto cell = cell_and_box.first;
        auto box = cell_and_box.second;
        if (cell != nullptr) {
            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
                Coordinate intersection = liang_barsky(start, target, box);
                //Get the edge that the intersection is on (with small margin)
                //left = 0, top = 1, right = 2, bottom = 3
                auto edge_index = -1;
                if (std::abs(intersection.x - box.left)<0.01) edge_index = 0;
                else if (std::abs(intersection.y - box.top)<0.01) edge_index = 1;
                else if (std::abs(intersection.x - box.getRight())<0.01) edge_index = 2;
                else if (std::abs(intersection.y - box.getBottom())<0.01) edge_index = 3;

                return {cell, box, edge_index};
            }
        }
        x += stepX;
        y += stepY;
    }
    return {nullptr, quadtree::Box(), -1};

}

Coordinate SimplePathPlanner::liang_barsky(Coordinate p1, Coordinate p2, quadtree::Box box) const{
    double t0 = 0.0;
    double t1 = 1.0;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;

    for (int edge = 0; edge < 4; edge++) {
        switch (edge) {
            case 0:
                p = -dx;
//                q = -(box.left - p1.x);
                q = p1.x - box.left;
                break;
            case 1:
                p = dx;
                q = box.getRight() - p1.x;
                break;
            case 2:
                p = -dy;
//                q = -(box.getBottom() - p1.y);
                q = p1.y - box.getBottom();
                break;
            case 3:
                p = dy;
                q = box.top- p1.y;
                break;
        }

        if (p == 0 && q < 0) return Coordinate{MAXFLOAT, MAXFLOAT};
        if (p == 0) continue;

        r = q / p;
        if (p < 0) {
            if (r > t1) return Coordinate{MAXFLOAT, MAXFLOAT};
            else if (r > t0) t0 = r;
        } else {
            if (r < t0) return Coordinate{MAXFLOAT, MAXFLOAT};
            else if (r < t1) t1 = r;
        }
    }

    auto t0x = p1.x + t0 * dx;
    auto t0y = p1.y + t0 * dy;
    auto t1x = p1.x + t1 * dx;
    auto t1y = p1.y + t1 * dy;

    if (t0 == 0.0) {
        t0x = MAXFLOAT;
        t0y = MAXFLOAT;
    } else if (t1 == 1.0) {
        t1x = MAXFLOAT;
        t1y = MAXFLOAT;
    }
    return t0x == MAXFLOAT ? Coordinate{t1x, t1y} : Coordinate{t0x, t0y};
}

std::pair<Coordinate, Coordinate> SimplePathPlanner::getEdgeCoordinates(quadtree::Box box, int edge_index) const {
    switch (edge_index) {
        case 0:
            return {Coordinate{box.left, box.top}, Coordinate{box.left, box.getBottom()}};
        case 1:
            return {Coordinate{box.left, box.top}, Coordinate{box.getRight(), box.top}};
        case 2:
            return {Coordinate{box.getRight(), box.top}, Coordinate{box.getRight(), box.getBottom()}};
        case 3:
            return {Coordinate{box.left, box.getBottom()}, Coordinate{box.getRight(), box.getBottom()}};
        default:
            return {Coordinate{MAXFLOAT, MAXFLOAT}, Coordinate{MAXFLOAT, MAXFLOAT}};
    }
}