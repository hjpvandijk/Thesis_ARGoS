//
// Created by hugo on 7-1-25.
//

#include "Algorithms.h"
#include "agent_implementation/agent.h"

std::vector<Coordinate> Algorithms::bresenhamLine(Agent* agent, Coordinate coordinate1, Coordinate coordinate2) {
    std::vector<Coordinate> points;
    double x1 = coordinate1.x;
    double y1 = coordinate1.y;
    double x2 = coordinate2.x;
    double y2 = coordinate2.y;
    double dx = abs(x2 - x1);
    double dy = abs(y2 - y1);
    double box_size = agent->quadtree->getSmallestBoxSize();
    double sx = (x2 > x1) ? box_size : -box_size;
    double sy = (y2 > y1) ? box_size : -box_size;
    double err = dx - dy;

    while (true) {
        points.emplace_back(Coordinate{x1, y1}); // Store the current point
        if (x1 == x2 && y1 == y2) break;

        double e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    return points;
}