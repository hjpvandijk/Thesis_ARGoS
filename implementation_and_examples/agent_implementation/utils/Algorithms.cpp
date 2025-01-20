//
// Created by hugo on 7-1-25.
//

#include "Algorithms.h"
#include "agent_implementation/agent.h"

/**
 * Adapted from https://www.cse.yorku.ca/~amana/research/grid.pdf?utm_source=chatgpt.com
 * @param agent
 * @param coordinate1
 * @param coordinate2
 * @return
 */
std::vector<Coordinate> Algorithms::Amanatides_Woo_Voxel_Traversal(Agent* agent, Coordinate coordinate1, Coordinate coordinate2) {
    double box_size = agent->quadtree->getSmallestBoxSize();
    std::vector<Coordinate> points;

    double x1 = coordinate1.x;
    double y1 = coordinate1.y;
    double x2 = coordinate2.x;
    double y2 = coordinate2.y;
    double x = floor(x1/box_size);
    double y = floor(y1/box_size);
    double xEnd = floor(x2/box_size);
    double yEnd = floor(y2/box_size);

    double dx = x2 - x1;
    double dy = y2 - y1;

    int stepX = dx > 0 ? 1 : -1;
    int stepY = dy > 0 ? 1 : -1;
    double tMaxX = (dx > 0 ? (floor(x1/box_size) + 1) * box_size - x1 : x1 - floor(x1/box_size) * box_size) / abs(dx);
    double tMaxY = (dy > 0 ? (floor(y1/box_size) + 1) * box_size - y1 : y1 - floor(y1/box_size) * box_size) / abs(dy);
    double tDeltaX = box_size / abs(dx);
    double tDeltaY = box_size / abs(dy);

    points.push_back(Coordinate{(x+0.5)*box_size, (y+0.5)*box_size});

    while (x != xEnd || y != yEnd) {
        if (tMaxX < tMaxY) {
            tMaxX += tDeltaX;
            x += stepX;
        } else {
            tMaxY += tDeltaY;
            y += stepY;
        }
        points.push_back(Coordinate{(x+0.5)*box_size, (y+0.5)*box_size});
    }


    return points;
}