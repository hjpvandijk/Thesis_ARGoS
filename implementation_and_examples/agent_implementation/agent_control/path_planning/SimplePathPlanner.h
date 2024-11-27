#ifndef IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H
#define IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H

#include <vector>
#include "agent_implementation/utils/coordinate.h"
#include "agent_implementation/utils/Quadtree.h"
#include <argos3/core/utility/math/vector2.h>

class Agent;

class SimplePathPlanner {
public:
//    /**
//     * Get the route from start to end
//     * Each vector is relative to the former
//     * @param start
//     * @param end
//     * @param quadtree
//     * @return
//     */
//    std::vector<argos::CVector2> getRoute(Agent* agent, Coordinate start, Coordinate end, const quadtree::Quadtree &quadtree, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle) const;
    std::vector<std::pair<Coordinate, Coordinate>> getRoute(Agent* agent, Coordinate start, Coordinate end) const;

private:
    std::vector<std::pair<Coordinate, Coordinate>> getRouteSections(Agent* agent, Coordinate start, Coordinate target) const;
    void getWallFollowingRoute(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route) const;
    bool directionToTargetFree(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, const std::vector<std::pair<Coordinate, Coordinate>> & route) const;

    std::vector<std::pair<quadtree::Quadtree::Cell*, int>> elegible_edges(quadtree::Quadtree::Cell * cell, int edge_index) const;

    std::tuple<quadtree::Quadtree::Cell *, quadtree::Box, int, double> rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate end) const;

    [[nodiscard]] Coordinate liang_barsky(Coordinate p1, Coordinate p2, quadtree::Box box) const;

    [[nodiscard]] std::pair<Coordinate, Coordinate> getEdgeCoordinates(quadtree::Box box, int edge_index) const;
};


#endif //IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H

