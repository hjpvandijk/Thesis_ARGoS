
#include "SimplePathPlanner.h"
#include "agent_implementation/utils/Box.h"
#include "agent_implementation/agent.h"


std::vector<std::pair<Coordinate, Coordinate>> SimplePathPlanner::getRoute(Agent* agent, Coordinate start, Coordinate target) const {
    return getRouteSections(agent, start, target);
}

std::vector<std::pair<Coordinate, Coordinate>> SimplePathPlanner::getRouteSections(Agent* agent, Coordinate start, Coordinate target) const {
    auto [cell, box, edge_index, _] = rayTraceQuadtreeOccupiedIntersection(agent, start, target);
    if (cell == nullptr) return {{start, target}};
    std::vector<std::pair<Coordinate, Coordinate>> route;
    auto [start_edge, end_edge] = getEdgeCoordinates(box, edge_index);
    auto edge_middle = Coordinate{(start_edge.x + end_edge.x) / 2, (start_edge.y + end_edge.y) / 2};

    //Line from agent to middle of the intersection edge
    route.emplace_back(agent->position, edge_middle);

    //Line from middle of the intersection edge to the correct side of the line, depending on the distance of that end to the agent
    auto distance_to_start = sqrt(pow(start_edge.x - agent->position.x, 2) + pow(start_edge.y - agent->position.y, 2));
    auto distance_to_end = sqrt(pow(end_edge.x - agent->position.x, 2) + pow(end_edge.y - agent->position.y, 2));
    if (distance_to_start >= distance_to_end) {
        route.emplace_back(edge_middle, start_edge);
    } else {
        route.emplace_back(edge_middle, end_edge);
    }

    getWallFollowingRoute(agent, cell, box.size, edge_index, target, route);

    //Append last point to target
    route.emplace_back(route.rbegin()->second, target);

    return route;
}

void SimplePathPlanner::getWallFollowingRoute(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route) const {
    //If the direction to the target is free from the cell, return
    if (directionToTargetFree(agent, cell, box_size, edge_index, target, route)) {
        //Delete the last full edge, and with a line to the middle of the last edge
        auto [begin_last_edge, end_last_edge] = *route.rbegin();
        auto edge_middle = Coordinate{(begin_last_edge.x + end_last_edge.x) / 2, (begin_last_edge.y + end_last_edge.y) / 2};
        route.erase(route.end() - 1);

        route.emplace_back(begin_last_edge, edge_middle);
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

        auto cell_to_edge_distance = sqrt(pow(edge_middle.x - agent->position.x, 2) + pow(edge_middle.y - agent->position.y, 2));
        auto [begin_last_edge, end_last_edge] = *route.rbegin();
        //If the end of the last edge is not the same as the start of the current edge, flip the edge, to keep the order correct
        if(end_last_edge == end_edge){
            //Flip start and end
            std::swap(start_edge, end_edge);
        }

        //If the edge (either way) is not already in the route, and the distance to the edge is the largest, save the cell and edge
        if (std::find(route.begin(), route.end(), std::pair{start_edge, end_edge}) == route.end() &&
            std::find(route.begin(), route.end(), std::pair{end_edge, start_edge}) == route.end() &&
            cell_to_edge_distance > max_distance) {
            bestCell = c;
            bestEdgeIndex = e;
            bestEdgeStart = start_edge;
            bestEdgeEnd = end_edge;
            max_distance = cell_to_edge_distance;
        }
    }

    if (bestCell != nullptr) {
        route.emplace_back(bestEdgeStart, bestEdgeEnd);
        getWallFollowingRoute(agent, bestCell, box_size, bestEdgeIndex, target, route);
    }

}

bool SimplePathPlanner::directionToTargetFree(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, const std::vector<std::pair<Coordinate, Coordinate>> & route) const{
    Coordinate start = cell->quadNode.coordinate;

    //Get the middle of the edge
    if (edge_index == 0) start.x = start.x - box_size / 2;
    else if (edge_index == 1) start.y = start.y + box_size / 2;
    else if (edge_index == 2) start.x = start.x + box_size / 2;
    else if (edge_index == 3) start.y = start.y - box_size / 2;

    //Find the intersection with the quadtree
    auto [intersection_cell, intersection_box, intersection_edge, distance_to_intersection] = rayTraceQuadtreeOccupiedIntersection(agent, start, target);

    if (intersection_cell != nullptr) {
        if (intersection_cell == cell)
            return false; //If we intersect the same cell, the line is going through the cell, so the direction is not free

        auto [start_edge, end_edge] = getEdgeCoordinates(intersection_box, intersection_edge);
        if (std::find(route.begin(), route.end(), std::pair{start_edge, end_edge}) ==
            route.end()) { //Can't go back to the same edge
            return false;
        }
        if (distance_to_intersection > box_size / 2) return true;
    }
    return true; //No intersection, so the direction is free

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
                if (cell->neighbors.at(i)->neighbors.at(edge_index) != nullptr) {
                    auto opposite_i = i < 2 ? i + 2 : i - 2;
                    edges.push_back({cell->neighbors.at(i)->neighbors.at(edge_index), opposite_i});
                } else {
                    edges.push_back({cell->neighbors.at(i), edge_index});
                }
            }
        }
    }
    return edges;
}

std::tuple<quadtree::Quadtree::Cell*, quadtree::Box, int, double> SimplePathPlanner::rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const {
    auto x = start.x;
    auto y = start.y;
    auto dx = target.x - start.x;
    auto dy = target.y - start.y;
    auto distance = sqrt(dx * dx + dy * dy);
    auto stepSize = 0.01;
    auto nSteps = std::ceil(distance / stepSize);
    auto stepX = dx / nSteps;
    auto stepY = dy / nSteps;

    //Already do one step to avoid the start cell
    x += stepX;
    y += stepY;
    for (int s = 1; s < nSteps; s++) {
        auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(Coordinate{x, y});
        auto cell = cell_and_box.first;
        auto box = cell_and_box.second;
        if (cell != nullptr) {
            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
                Coordinate intersection = liang_barsky(start, target, box);
                //Get the edge that the intersection is on (with small margin)
                //left = 0, top = 1, right = 2, bottom = 3
                auto edge_index = -1;
                if (std::abs(intersection.x - box.left)<0.00001) edge_index = 0;
                else if (std::abs(intersection.y - box.top)<0.00001) edge_index = 1;
                else if (std::abs(intersection.x - box.getRight())<0.00001) edge_index = 2;
                else if (std::abs(intersection.y - box.getBottom())<0.00001) edge_index = 3;

                auto dist_to_intersection = sqrt(pow(intersection.x - start.x, 2) + pow(intersection.y - start.y, 2));

                return {cell, box, edge_index, dist_to_intersection};
            }
        }
        x += stepX;
        y += stepY;
    }
    return {nullptr, quadtree::Box(), -1, 0};

}

/**
 * From the python implementation of Liang-Barsky: https://www.geeksforgeeks.org/liang-barsky-algorithm/
 * @param p1
 * @param p2
 * @param box
 * @return
 */
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