#include <iostream>
#include <vector>
#include "coordinate.h"
#include "quadtree.h"
#include <argos3/core/utility/logging/argos_log.h>

#define FINEST_NODE_SIZE 0.1

#include <iostream>
#include <fstream>

void Quadtree::insert(Coordinate p, Occupancy occupancy) {
    this->root = insert(this->root, p, occupancy);
}

bool Quadtree::search(Coordinate p) {
    return search(this->root, p);
}

Node *Quadtree::insert(Node *node, Coordinate p, Occupancy occupancy) {
    if (!node) {
        return new Node(p, occupancy);
    }

    // Calculate the mid-point
    double midX = (this->topLeft.x + this->bottomRight.x) / 2;
    double midY = (this->topLeft.y + this->bottomRight.y) / 2;
//    double midX = (node->NW->pos.x + node->NE->pos.x) / 2;
//    double midY = (node->NW->pos.y + node->SW->pos.y) / 2;
    argos::LOG << "insert: " << midX << " " << midY << std::endl;

    if (p.x >= midX && p.y >= midY) {
        node->NW = insert(node->NW, p, occupancy);
    } else if (p.x < midX && p.y >= midY) {
        node->NE = insert(node->NE, p, occupancy);
    } else if (p.x <= midX && p.y < midY) {
        node->SW = insert(node->SW, p, occupancy);
    } else {
        node->SE = insert(node->SE, p, occupancy);
    }
    //Unknown because there are subnodes
    node->occupancy = UNKNOWN;

    return node;
}



bool Quadtree::search(Node *node, Coordinate p) {
    if (!node) {
        return false;
    }

    if (node->pos.x == p.x && node->pos.y == p.y) {
        return true;
    }

    // Calculate the mid-point
    double midX = (this->topLeft.x + this->bottomRight.x) / 2;
    double midY = (this->topLeft.y + this->bottomRight.y) / 2;
//    double midX = (node->NW->pos.x + node->NE->pos.x) / 2;
//    double midY = (node->NW->pos.y + node->SW->pos.y) / 2;

    if (p.x >= midX && p.y >= midY) {
        return search(node->NW, p);
    } else if (p.x < midX && p.y >= midY) {
        return search(node->NE, p);
    } else if (p.x <= midX && p.y < midY) {
        return search(node->SW, p);
    } else {
        return search(node->SE, p);
    }
}

//std::string Quadtree::toStringHelper(Node* node, int depth) {
//    if (!node) {
//        return "";
//    }
//
//    std::string s(depth, ' ');
//    s += "(" + node->pos.toString() + ");";
//
//    s += toStringHelper(node->NW, depth + 1);
//    s += toStringHelper(node->NE, depth + 1);
//    s += toStringHelper(node->SW, depth + 1);
//    s += toStringHelper(node->SE, depth + 1);
//
//    return s;
//}

std::string Quadtree::toStringHelper(Node* node, int depth) {
    if (!node) {
        return "";
    }

    std::string s(depth * 2, ' ');  // Multiply depth by 2 to make the output wider
    s += "+ (" + std::to_string(node->pos.x) + ", " + std::to_string(node->pos.y) + ")\n";

    // Print lines connecting the node to its children
    if (node->NW || node->NE || node->SW || node->SE) {
        s += std::string(depth * 2, ' ') + "|";
        if (node->NW || node->NE) {
            s += "-- NW:NE\n";
        }
        if (node->SW || node->SE) {
            s += std::string(depth * 2, ' ') + "|";
            s += "-- SW:SE\n";
        }
    }

    s += toStringHelper(node->NW, depth + 1);
    s += toStringHelper(node->NE, depth + 1);
    s += toStringHelper(node->SW, depth + 1);
    s += toStringHelper(node->SE, depth + 1);

    return s;
}

std::string Quadtree::toString() {
    return toStringHelper(root, 0) + "\n";
}

void Quadtree::writeToFile(std::string filename) {
    std::ofstream myfile;
    myfile.open (filename + ".txt");
    myfile << this->toString();
    myfile.close();
}


//int main() {
//    Coordinate topLeft = {0, 0};
//    Coordinate bottomRight = {10, 10};
//    Quadtree qt(topLeft, bottomRight);
//
//    qt.insert({5, 5});
//    qt.insert({2, 3});
//    qt.insert({7, 8});
//
//    std::cout << "Search (5,5): " << qt.search({5, 5}) << std::endl;
//    std::cout << "Search (1,1): " << qt.search({1, 1}) << std::endl;
//
//    return 0;
//}
