//
// Created by hugo on 22-6-24.
//

#ifndef ARGOS3_EXAMPLES_QUADTREE_H
#define ARGOS3_EXAMPLES_QUADTREE_H

#include <iostream>
#include <vector>
#include "coordinate.h"

enum Occupancy {
    UNKNOWN,
    FREE,
    OCCUPIED,
};


struct Node {
    Coordinate pos;
    Node* NW;
    Node* NE;
    Node* SW;
    Node* SE;
    Occupancy occupancy;

    Node(Coordinate p, Occupancy occ) : pos(p), NW(nullptr), NE(nullptr), SW(nullptr), SE(nullptr), occupancy(occ) {}
};

class Quadtree {
public:
    Quadtree(Coordinate topLeft, Coordinate bottomRight)
            : topLeft(topLeft), bottomRight(bottomRight), root(nullptr) {}

    void insert(Coordinate p, Occupancy occupancy);

    bool search(Coordinate p);

    std::string toString();

    void writeToFile(std::string filename);

private:
    Coordinate topLeft;
    Coordinate bottomRight;
    Node* root;

    Node* insert(Node* node, Coordinate p, Occupancy occupancy);

    bool search(Node* node, Coordinate p);

    std::string toStringHelper(Node *node, int depth);

};

#endif //ARGOS3_EXAMPLES_QUADTREE_H
