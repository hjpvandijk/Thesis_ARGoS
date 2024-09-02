#pragma once

#include <cassert>
#include <algorithm>
#include <array>
#include <memory>
#include <type_traits>
#include <vector>
#include "Box.h"
#include <iostream>
#include <fstream>

//Adapted from https://github.com/pvigier/Quadtree

namespace quadtree {

    enum Occupancy {
        UNKNOWN,
        FREE,
        OCCUPIED,
        ANY
    };


    struct QuadNode {
        Coordinate coordinate;
        Occupancy occupancy;
        double visitedAtS;

        bool operator==(const QuadNode &rhs) const {
            return coordinate.x == rhs.coordinate.x && coordinate.y == rhs.coordinate.y;
        }
    };

    class Quadtree {

    public:
        Quadtree(const Box &box) :
                mBox(box), mRoot(std::make_unique<Node>()) {
            mRoot->values.push_back(QuadNode{box.getCenter(), UNKNOWN, 0});
        }

        /**
         * @brief Add a coordinate to the quadtree with the given occupancy
         * @param coordinate
         * @param occupancy
         */
        void add(Coordinate coordinate, Occupancy occupancy, double visitedAtS) {
            auto node = QuadNode{coordinate, occupancy, visitedAtS};
            add(node);
        }

        /**
         * @brief Add a QuadNode to the quadtreee
         * @param value
         */
        void add(const QuadNode &value) {
            add(mRoot.get(), 0, mBox, value);
        }

        void remove(const QuadNode &value) {
            remove(mRoot.get(), mBox, value);
        }

        /**
         * Returns all the occupied QuadNodes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<QuadNode> queryOccupied(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return query(box, OCCUPIED);
        }


        /**
         * Returns all the occupied boxes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<Box> queryOccupiedBoxes(Coordinate coordinate, double areaSize, double currentTimeS) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return queryBoxes(box, OCCUPIED, currentTimeS);
        }

        /**
         * Returns all the unexplored boxes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<Box> queryUnexploredBoxes(Coordinate coordinate, double areaSize, double currentTimeS) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return queryBoxes(box, UNKNOWN, currentTimeS);
        }

        /**
         * Returns all the frontier boxes surrounding the given coordinate within the given area size
         */
        std::vector<Box> queryFrontierBoxes(Coordinate coordinate, double areaSize, double currentTimeS) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            //A cell is a frontier iff (according to dynamic frontier-led swarming):
            //1. Occupancy = explored
            //2. At least one neighbor is unexplored using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

            //Get all the explored but free boxes
            std::vector<Box> exploredBoxes = queryBoxes(box, FREE, currentTimeS);
//

            //Get all the frontier boxes
            std::vector<Box> frontierBoxes = {};
            for (Box exploredBox: exploredBoxes) {
                //Get the 8-connected Moore neighbours of the explored box
                if (isMooreNeighbourUnknown(exploredBox)) {
                    frontierBoxes.push_back(exploredBox);
                }

            }

            return frontierBoxes;

        }

        /**
         * Find if at least one of the 8-connected moore neighboring quadnodes of a given box is unexplored.
         * @param box
         * @return
         */
        bool isMooreNeighbourUnknown(const Box &box) const {
//            argos::LOG << "Checking moore neighbours of box: " << box.getCenter().x << " " << box.getCenter().y << " of size " << box.getSize() << std::endl;
            //See if coordinate to the left is in the quadtree and get its occupancy
            Coordinate left = Coordinate{box.getCenter().x - box.size, box.getCenter().y};
            if (mBox.contains(left)) {
//                argos::LOG << "left: " << left.x << " " << left.y << std::endl;
                if (isCoordinateUnknown(left)) {
                    return true;
                }
            }

            //See if coordinate to the right is in the quadtree and get its occupancy
            Coordinate right = Coordinate{box.getCenter().x + box.size, box.getCenter().y};
            if (mBox.contains(right)) {
//                argos::LOG << "right: " << right.x << " " << right.y << std::endl;
                if (isCoordinateUnknown(right)) {
                    return true;
                }
            }

            //See if coordinate to the top is in the quadtree and get its occupancy
            Coordinate top = Coordinate{box.getCenter().x, box.getCenter().y + box.size};
            if (mBox.contains(top)) {
//                argos::LOG << "top: " << top.x << " " << top.y << std::endl;
                if (isCoordinateUnknown(top)) {
                    return true;
                }
            }

            //See if coordinate to the bottom is in the quadtree and get its occupancy
            Coordinate bottom = Coordinate{box.getCenter().x, box.getCenter().y - box.size};
            if (mBox.contains(bottom)) {
//                argos::LOG << "bottom: " << bottom.x << " " << bottom.y << std::endl;
                if (isCoordinateUnknown(bottom)) {
                    return true;
                }
            }

            //See if coordinate to the top left is in the quadtree and get its occupancy
            Coordinate topLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y + box.size};
            if (mBox.contains(topLeft)) {
//                argos::LOG << "topLeft: " << topLeft.x << " " << topLeft.y << std::endl;
                if (isCoordinateUnknown(topLeft)) {
                    return true;
                }
            }

            //See if coordinate to the top right is in the quadtree and get its occupancy
            Coordinate topRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y + box.size};
            if (mBox.contains(topRight)) {
//                argos::LOG << "topRight: " << topRight.x << " " << topRight.y << std::endl;
                if (isCoordinateUnknown(topRight)) {
                    return true;
                }
            }

            //See if coordinate to the bottom left is in the quadtree and get its occupancy
            Coordinate bottomLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y - box.size};
            if (mBox.contains(bottomLeft)) {
//                argos::LOG << "bottomLeft: " << bottomLeft.x << " " << bottomLeft.y << std::endl;
                if (isCoordinateUnknown(bottomLeft)) {
                    return true;
                }
            }

            //See if coordinate to the bottom right is in the quadtree and get its occupancy
            Coordinate bottomRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y - box.size};
            if (mBox.contains(bottomRight)) {
//                argos::LOG << "bottomRight: " << bottomRight.x << " " << bottomRight.y << std::endl;
                if (isCoordinateUnknown(bottomRight)) {
                    return true;
                }
            }

            return false;

        }

        /**
         * Returns QuadNodes that intersect with or are contained by the given box
         * @param box
         * @param occupancy
         * @return
         */

        std::vector<QuadNode> query(const Box &box, Occupancy occupancy) const {
            auto queried_values = std::vector<QuadNode>();
            query(mRoot.get(), mBox, box, queried_values, occupancy);
            return queried_values;
        }

        /**
         * Returns all the values that intersect with or are contained by given box
         * @param box
         * @param occupancy
         * @return
         */
        std::vector<Box> queryBoxes(const Box &box, Occupancy occupancy, double currentTimeS) const {
            auto boxes = std::vector<Box>();
            queryBoxes(mRoot.get(), mBox, box, boxes, occupancy, currentTimeS);
            return boxes;
        }

        /**
         * Returns the QuadNode containing the coordinate
         * @param coordinate
         */
        std::vector<Occupancy> getOccupanciesFromCoordinate(Coordinate coordinate) const {
            auto QuadNodes = std::vector<QuadNode>();
            getQuadNodesFromCoordinate(mRoot.get(), mBox, coordinate, QuadNodes);
            std::vector<Occupancy> occupancies;
            for(auto node: QuadNodes)
                occupancies.push_back(node.occupancy);
            return occupancies;
        }

        /**
         * Returns the QuadNode containing the coordinate
         * @param coordinate
         */
        bool isCoordinateUnknown(Coordinate coordinate) const {
            auto QuadNodes = std::vector<QuadNode>();
            getQuadNodesFromCoordinate(mRoot.get(), mBox, coordinate, QuadNodes);
//            argos::LOG << "Quadnodes size: " << QuadNodes.size() << std::endl;
            for(auto node: QuadNodes) {
//                argos::LOG << "occ: " << occ << std::endl;
                if(node.occupancy == UNKNOWN)
                    return true;
            }
            return false;
        }

        /**
         * @brief Find all intersections between values stored in the quadtree
         * @return
         */
        std::vector<std::pair<QuadNode, QuadNode>> findAllIntersections() const {
            auto intersections = std::vector<std::pair<QuadNode, QuadNode>>();
            findAllIntersections(mRoot.get(), intersections);
            return intersections;
        }

        Box getBox() const {
            return mBox;
        }

        /**
         * @brief Export the quadtree to a file
         * @param filename
         */
        void exportQuadtreeToFile(const std::string &filename) {
            std::ofstream file(filename + ".txt");
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return;
            }

            std::function<void(const Node *, const Box &, int)> traverse;
            traverse = [&](const Node *node, const Box &box, int depth) {
                if (node == nullptr) return;

                file << box.left << " " << box.top << " " << box.size << " " << "\n";

                // Write the bounding box, occupancy and depth of this node to the file
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                for (const auto &value: node->values) {
                    file << box.left << " " << box.top << " " << box.size << " " << value.occupancy << " " << "\n";
                }

                // Traverse the children
                for (int i = 0; i < 4; ++i) {
                    if (node->children[i]) {
                        traverse(node->children[i].get(), computeBox(box, i), depth + 1);
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0);
            file.close();
        }

        /**
         * @brief Create a vector of strings from the quadtree
         */
        void toStringVector(std::vector<std::string> *strings) {
            std::function<void(const Node *, const Box &, int)> traverse;
            traverse = [&](const Node *node, const Box &box, int depth) {
                if (node == nullptr) return;

                bool allSameOccupancy = false;

                // Write the bounding box, occupancy and depth of this node to the file
                for (const auto &value: node->values) {
                    QuadNode curQuadNode = value;
                    // If the occupancy is ANY, we don't need to store it, as the children will have new info
                    // If the occupancy is UNKNOWN, we don't need to store it, as a child not existing will also yield in an UNKNOWN
                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
                        continue;
                    // If the occupancy is OCCUPIED or FREE, we want to exchange that information. And we don't have to send any children as they will be all the same.
                    if (value.occupancy == OCCUPIED || value.occupancy == FREE)
                        allSameOccupancy = true;
                    std::string str =
                            std::to_string(box.getCenter().x) + ';' + std::to_string(box.getCenter().y) + ':' +
                            std::to_string(curQuadNode.occupancy) + '@' + std::to_string(curQuadNode.visitedAtS);
                    strings->emplace_back(str);
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    // Traverse the children
                    for (int i = 0; i < 4; ++i) {
                        if (node->children[i]) {
                            traverse(node->children[i].get(), computeBox(box, i), depth + 1);
                        }
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0);
        }

        /**
         * Get all the boxes in the quadtree
         * @return
         */
        std::vector<std::tuple<Box, int, double>> getAllBoxes() {
            std::vector<std::tuple<Box, int, double>> boxesAndOccupancyAndTicks = {};
            std::function<void(const Node *, const Box &, int, std::vector<std::tuple<Box, int, double>> *)> traverse;
            traverse = [&](const Node *node, const Box &box, int depth,
                           std::vector<std::tuple<Box, int, double>> *boxesAndOccupancyAndTicks) {
                if (node == nullptr) return;
                bool allSameOccupancy = false;
                assert(node->values.empty() || node->values.size() == 1);
                for (const auto &value: node->values) {
                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
                        continue;
                    // If the occupancy is OCCUPIED or FREE, we want to exchange that information. And we don't have to send any children as they will be all the same.
                    if (value.occupancy == OCCUPIED || value.occupancy == FREE)
                        allSameOccupancy = true;

                    boxesAndOccupancyAndTicks->emplace_back(std::tuple(box, value.occupancy, value.visitedAtS));
                }

                if(!node->values.empty() && node->values.front().occupancy != UNKNOWN && node->values.front().occupancy != ANY){
                    if(!isLeaf(node)) {
                        for(const auto & i : node->children) {
                            if(i->values.empty())
                                argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line GetAllBoxes" << std::endl;
                        }
                    }
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    for (int i = 0; i < 4; ++i) {
                        if (node->children[i]) {
                            traverse(node->children[i].get(), computeBox(box, i), depth + 1, boxesAndOccupancyAndTicks);
                        }
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0, &boxesAndOccupancyAndTicks);


            return boxesAndOccupancyAndTicks;

        }

        double getMinSize() {
            return this->MinSize;
        }

        double getSmallestBoxSize() {
            return this->Smallest_Box_Size;
        }


    private:
        static constexpr auto Threshold = std::size_t(16);
        static constexpr double MinSize = 0.2;
        double Smallest_Box_Size = MinSize;
        static constexpr double EvaporationTime = 100.0;


        struct Node {
            std::array<std::unique_ptr<Node>, 4> children;
            std::vector<QuadNode> values;
        };

        Box mBox;
        std::unique_ptr<Node> mRoot;

        /**
         * @brief Check if the given node is a leaf i.e. had no children
         * @param node
         * @return
         */
        bool isLeaf(const Node *node) const {
            return !static_cast<bool>(node->children[0]);
        }

        /**
         * @brief Compute the box of the i-th child of the given box
         * @param box
         * @param i
         * @return
         */
        Box computeBox(const Box &box, int i) const {
            auto origin = box.getTopLeft();
            auto childSize = box.getSize() / 2.0;
            switch (i) {
                // North West
                case 0:
                    return Box(origin, childSize);
                    // Norst East
                case 1:
                    return Box(Coordinate{origin.x + childSize, origin.y}, childSize);
                    // South West
                case 2:
                    return Box(Coordinate{origin.x, origin.y - childSize}, childSize);
                    // South East
                case 3:
                    return Box(Coordinate{origin.x + childSize, origin.y - childSize}, childSize);
                default:
                    assert(false && "Invalid child index");
                    return Box();
            }
        }

        /**
         * @brief Get the quadrant which the given box should go in in the node box
         * @param nodeBox
         * @param valueBox
         * @return
         */
        int getQuadrant(const Box &nodeBox, const Box &valueBox) const {
            auto center = nodeBox.getCenter();

            // West
            if (valueBox.getRight() < center.x) {
                // North West
                if (valueBox.getBottom() > center.y)
                    return 0;
                    // South West
                else if (valueBox.top <= center.y)
                    return 2;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // East
            else if (valueBox.left >= center.x) {
                // North East
                if (valueBox.getBottom() > center.y)
                    return 1;
                    // South East
                else if (valueBox.top <= center.y)
                    return 3;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // Not contained in any quadrant
            else
                return -1;
        }

        /**
         * @brief Get the quadrant which the given coordinate should go in in the node box
         * @param nodeBox
         * @param valueCoordinate
         * @return
         */
        int getQuadrant(const Box &nodeBox, const Coordinate &valueCoordinate) const {
            auto center = nodeBox.getCenter();

            //If the value is the same as the center, it is not contained in any quadrant
            if(center == valueCoordinate)
                return 4;

            // West
            if (valueCoordinate.x < center.x) {
                // North West
                if (valueCoordinate.y > center.y)
                    return 0;
                    // South West
                else if (valueCoordinate.y <= center.y)
                    return 2;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // East
            else if (valueCoordinate.x >= center.x) {
                // North East
                if (valueCoordinate.y > center.y)
                    return 1;
                    // South East
                else if (valueCoordinate.y <= center.y)
                    return 3;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // Not contained in any quadrant
            else
                return -1;
        }

        /**
         * @brief Add a value to the quadtree
         * @param node
         * @param depth
         * @param box
         * @param value
         */
        void add(Node *node, std::size_t depth, const Box &box, const QuadNode &value) {
            assert(node != nullptr);
            assert(box.contains(value.coordinate));

            assert(value.occupancy != ANY && "Added occupancy should not be ANY");
            assert(value.occupancy != UNKNOWN && "Added occupancy should not be UNKNOWN");

            if (isLeaf(node)) {
                // Insert the value in this node if possible

                //If the box size is the minimum size we allow (corresponding to finest mapping level),
                // then we only contain a single QuadNode. Update the occupancy of this node to the most important occupancy.
                if (box.size <= MinSize) {
                    if(box.size <= Smallest_Box_Size) Smallest_Box_Size = box.size;

                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (node->values.empty()) {
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                    } else {

                        if (node->values.front().occupancy == FREE || value.occupancy == FREE)
                            newNode.occupancy = FREE;
                        if (node->values.front().occupancy == OCCUPIED || value.occupancy == OCCUPIED)
                            newNode.occupancy = OCCUPIED;

                        newNode.visitedAtS = std::max(node->values.front().visitedAtS, value.visitedAtS);


                    }
                    assert(newNode.occupancy == FREE || newNode.occupancy == OCCUPIED && "new node occupancy should be FREE or OCCUPIED");
                    // Make the only value the 'merged node'
                    node->values.clear();
                    node->values.push_back(newNode);
                }
                    // Otherwise, we split and we try again
                else {
                    //If the to be added occupancy is the same as the parent, and the visited time is not too far apart, we can skip adding.
                    if (node->values.empty() || !(value.occupancy == node->values.front().occupancy && value.visitedAtS - node->values.front().visitedAtS <=10)) {
                        split(node, box);
                        add(node, depth, box, value);
                    }


                }
            } else {
                // If the node is not a leaf
                // And if the box center is the same as the value coordinate (meaning this value information is the same for all children of this node),
                // then we only contain a single QuadNode. Update the occupancy of this node to the most important occupancy.
                // This should only happen when adding nodes received from other agents.
                if (box.getCenter() == value.coordinate) {
                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (node->values.empty()) {
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                    } else {

                        if (node->values.front().occupancy == FREE || value.occupancy == FREE)
                            newNode.occupancy = FREE;
                        if (node->values.front().occupancy == OCCUPIED || value.occupancy == OCCUPIED)
                            newNode.occupancy = OCCUPIED;


                        newNode.visitedAtS = std::max(node->values.front().visitedAtS, value.visitedAtS);

                    }
                    // Make the only value the 'merged node'
                    node->values.clear();
                    node->values.push_back(newNode);
                    //Unitialize the children nodes if there are any
                    for (auto &child: node->children)
                        child.reset();
                    assert(isLeaf(node) && "The node should be a leaf again now");
                // Else we add the value to the appropriate child
                } else {
                    auto i = getQuadrant(box, value.coordinate);
                    // Add the value in a child if the value is entirely contained in it
                    assert(i != -1 && "A value should be contained in a quadrant");
                    assert(i != 4 && "A value should not be the same as the center of the box");
                    Box subbox = computeBox(box, i);
                    add(node->children[static_cast<std::size_t>(i)].get(), depth + 1, computeBox(box, i), value);

//                Check if all children have the same occupancy
                    Occupancy firstOccupancy = UNKNOWN;
                    bool allSameOccupancy = true;
                    bool visitedTimesTooFarApart = false;
                    double minVisitedTime = MAXFLOAT;
                    double maxVisitedTime = -1;

                    if (node->children[0]->values.empty())
                        allSameOccupancy = false;
                    else {
                        //Get the occupancy of the first child (if it is empty, it is not the same occupancy as the others)
                        firstOccupancy = node->children[0]->values.front().occupancy;
                        for (const auto &child: node->children) {
                            //If a child is empty, it is not the same occupancy as the others
                            //If a child has a different occupancy than the first child, it is not the same occupancy as all others
                            //If a child has occupancy ANY, further nested nodes will have a different occupancy.
                            if (child->values.empty() || child->values.front().occupancy == ANY ||
                                child->values.front().occupancy != firstOccupancy) {
                                allSameOccupancy = false;
                                break;
                            }
                        //If the visited time of the child is too far apart from the first child, it is not the same occupancy as the others
                            if(child->values.front().visitedAtS < minVisitedTime)
                                minVisitedTime = child->values.front().visitedAtS;
                            if(child->values.front().visitedAtS > maxVisitedTime)
                                maxVisitedTime = child->values.front().visitedAtS;
                        }
                        //If the visited times are too far apart, the children should be kept
                        if (maxVisitedTime - minVisitedTime > 10.0)
                            visitedTimesTooFarApart = true;
                    }
//                assert(!node->values.empty() && "A non-leaf node should have a value");

                    // If all children have the same occupancy, and their visited times are not too far apart, we can delete the children and the parents will have all info
                    if(allSameOccupancy && !visitedTimesTooFarApart) {

                        //Unitialize the children nodes as the parent now contains their information.
                        for (auto &child: node->children)
                            child.reset();
                        assert(isLeaf(node) && "The node should be a leaf again now");

                        //If all children have the same occupancy, give the parent that occupancy
                        node->values.front().occupancy = firstOccupancy;
                        assert(node->values.front().occupancy != UNKNOWN && "A non-leaf node should not have UNKNOWN occupancy");
//                        argos::LOG << "Parent node occupancy: " << node->values.front().occupancy << std::endl;
                        node->values.front().visitedAtS = maxVisitedTime;

                    } else {
                        //If the children have different occupancies, or the visited times are too far apart, the parent should have occupancy ANY
                        node->values.front().occupancy = ANY;

                    }
//
//
                }

            }
            //CHECKING
//            argos::LOGERR << "Checking node values line 649" << std::endl;
            if(!node->values.empty() && node->values.front().occupancy != UNKNOWN && node->values.front().occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->values.empty())
                            argos::LOGERR << "leaf value empty while parent is " << node->values.front().occupancy << " 651" << std::endl;
                    }
                }
            }
        }

        /**
         * @brief Split a leaf node into four children
         * @param node
         * @param box
         */
        void split(Node *node, const Box &box) {
            assert(node != nullptr);
            assert(isLeaf(node) && "Only leaves can be split");
            // Create children
//            argos::LOG << node->children.size() << std::endl;
//            argos::LOG << node->children[0]->children.size() << std::endl;
//            auto newValues = std::vector<QuadNode>(); // New values for this node



            for (auto &child: node->children)
                child = std::make_unique<Node>();

            assert(!isLeaf(node) && "A node should not be a leaf after splitting");

            //If node has occupancy FREE or OCCUPIED, it entails all its children are also of that value.
            //When adding a new coordinate and occupancy to that parent, we should set the three remaining children to the parent occupancy.
            if(!node->values.empty() && (node->values.front().occupancy == FREE || node->values.front().occupancy == OCCUPIED)) {
                for (int i = 0; i < node->children.size(); i++) {
                    Coordinate coordinate{};
                    Coordinate boxCenter = box.getCenter();
                    double childSize = box.getSize() / 2.0;

                    if (i == 0) //North West
                        coordinate = {boxCenter.x - childSize, boxCenter.y + childSize};
                    if (i == 1) //North East
                        coordinate = {boxCenter.x + childSize, boxCenter.y + childSize};
                    if (i == 2) //South West
                        coordinate = {boxCenter.x - childSize, boxCenter.y - childSize};
                    if (i == 3) //South East
                        coordinate = {boxCenter.x + childSize, boxCenter.y - childSize};

                    auto quadNode = QuadNode{coordinate, node->values.front().occupancy,
                                             node->values.front().visitedAtS};
                    node->children[i]->values.push_back(quadNode);
                }
            }
//
//          //Shouldn't assign anything to the children yet, the new coordinate will be added after this.
//            // Assign values to children
//            for (const auto &value: node->values) {
//                auto i = getQuadrant(box, value.coordinate);
//                assert(i != -1 && "A value should be contained in a quadrant");
//                node->children[static_cast<std::size_t>(i)]->values.push_back(value);
//            }
//            node->values = std::move(newValues);
            node->values.clear();
            node->values.push_back(QuadNode{box.getCenter(), ANY, 0});
        }
        /**
         * @brief Remove a value from the quadtree
         * @param node
         * @param box
         * @param value
         * @return
         */
        bool remove(Node *node, const Box &box, const QuadNode &value) {
            assert(node != nullptr);
            assert(box.contains(value.coordinate));
            if (isLeaf(node)) {
                // Remove the value from node
                removeValue(node, value);
                return true;
            } else {
                // Remove the value in a child if the value is entirely contained in it
                auto i = getQuadrant(box, value.coordinate);
                if (i != -1) {
                    if (remove(node->children[static_cast<std::size_t>(i)].get(), computeBox(box, i), value))
                        return tryMerge(node);
                }
                    // Otherwise, we remove the value from the current node
                else
                    removeValue(node, value);
                return false;
            }
        }

        void removeValue(Node *node, const QuadNode &value) {
            // Find the value in node->values
            auto it = std::find_if(std::begin(node->values), std::end(node->values),
                                   [this, &value](const auto &rhs) { return value == rhs; });
            assert(it != std::end(node->values) && "Trying to remove a value that is not present in the node");
            // Swap with the last element and pop back
            *it = std::move(node->values.back());
            node->values.pop_back();
        }

        bool tryMerge(Node *node) {
            assert(node != nullptr);
            assert(!isLeaf(node) && "Only interior nodes can be merged");
            auto nbValues = node->values.size();
            for (const auto &child: node->children) {
                if (!isLeaf(child.get()))
                    return false;
                nbValues += child->values.size();
            }
            if (nbValues <= Threshold) {
                node->values.reserve(nbValues);
                // Merge the values of all the children
                for (const auto &child: node->children) {
                    for (const auto &value: child->values)
                        node->values.push_back(value);
                }
                // Remove the children
                for (auto &child: node->children)
                    child.reset();
                return true;
            } else
                return false;
        }

        /**
         * @brief Query the quadtree for QuadNodes that intersect with or are contained by the given box
         * @param node
         * @param box
         * @param queryBox
         * @param values
         * @param occupancy
         */
        void query(Node *node, const Box &box, const Box &queryBox, std::vector<QuadNode> &values,
                   Occupancy occupancy) const {
            assert(node != nullptr);
            assert(queryBox.intersects_or_contains(box));
            assert(node->values.empty() || node->values.size()==1);
            for (const auto &value: node->values) {
                if ((occupancy == ANY || value.occupancy == occupancy) &&
                    (queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
                    values.push_back(value);
            }
            if(!node->values.empty() && node->values.front().occupancy != UNKNOWN && node->values.front().occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->values.empty())
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 771" << std::endl;
                    }
                }
            }
            //Only check further if the occupancy of the non-leaf node is not all the same for its children, so UNKNOWN.
            if (!isLeaf(node) && (node->values.empty() || node->values.begin()->occupancy==ANY || node->values.begin()->occupancy==UNKNOWN)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
                    if (queryBox.intersects_or_contains(childBox))
                        query(node->children[i].get(), childBox, queryBox, values, occupancy);
                }
            }
        }
        /**
         * @brief Query the quadtree for boxes that intersect with or are contained by the given box
         * @param node
         * @param box
         * @param queryBox
         * @param boxes
         * @param occupancy
         */
        void queryBoxes(Node *node, const Box &box, const Box &queryBox, std::vector<Box> &boxes,
                        Occupancy occupancy, double currentTimeS) const {
            assert(node != nullptr);
            assert(queryBox.intersects_or_contains(box));
            assert(node->values.empty() || node->values.size()==1);
            for (auto &value: node->values) {
                //Check if pheromone is expired, if so, set the occupancy to unknown
                //TODO: Remove the node to save memory
                if(value.occupancy==Occupancy::FREE && calculatePheromone(value.visitedAtS, currentTimeS)<0.05){
                    value.occupancy = Occupancy::UNKNOWN;
                }


                if (value.occupancy == occupancy &&
                    (queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
                    boxes.push_back(box);
            }
            if(!node->values.empty() && node->values.front().occupancy != UNKNOWN && node->values.front().occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->values.empty())
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 813" << std::endl;
                    }
                }
            }
            //Only check further if the occupancy of the non-leaf node is not all the same for its children, so ANY.
            if (!isLeaf(node) && (node->values.empty() || node->values.begin()->occupancy==ANY || node->values.begin()->occupancy==UNKNOWN)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
                    if (queryBox.intersects_or_contains(childBox))
                        queryBoxes(node->children[i].get(), childBox, queryBox, boxes, occupancy, currentTimeS);
                }
            }
        }

        double calculatePheromone(double visitedTime, double currentTime) const {
            double pheromone = 1.0-std::min((currentTime - visitedTime)/EvaporationTime, 1.0);
            return pheromone;
        }



        /**
         * @brief Get the QuadNode that contains the given coordinate
         * @param node
         * @param box
         * @param queryCoordinate
         * */
        void getQuadNodesFromCoordinate(Node *node, const Box &box, const Coordinate &queryCoordinate, std::vector<QuadNode> & QuadNodes) const {
            assert(node != nullptr);
            assert(box.contains(queryCoordinate));

            assert(node->values.empty() || node->values.size()==1);
            //If it is a leaf node, return the QuadNode if it exists. If it does not exist, it means this coordinate is unexplored.
            if (isLeaf(node)) {
                if (node->values.empty()) {
//                    argos::LOG << "Yes it is because of this" << std::endl;
                    QuadNodes.push_back(QuadNode{queryCoordinate, UNKNOWN, 0});
                } else {
                    assert(node->values.front().occupancy != ANY && "leaf occupancy should never be ANY");
                    QuadNodes.push_back(node->values.front());
                }
                // If it is not a leaf node, find the nested nodes, and search them.
            } else {
                //If the node occupancy is ANY or UNKNOWN, there can be nested nodes with different occupancies
                if(node->values.empty() || node->values.front().occupancy ==ANY || node->values.front().occupancy == UNKNOWN) {
                    auto i = getQuadrant(box, queryCoordinate);
                    //If i=4, so the query coordinate is the exact center, check all children
                    if(i==4) {
                        for(int j=0; j<node->children.size(); j++) {
                            auto childBox = computeBox(box, static_cast<int>(j));

                            getQuadNodesFromCoordinate(node->children[j].get(), childBox, queryCoordinate, QuadNodes);
                        }
                    } else {
                        auto childBox = computeBox(box, static_cast<int>(i));

                        getQuadNodesFromCoordinate(node->children[i].get(), childBox, queryCoordinate, QuadNodes);
                    }
                //Else the nested nodes have the same occupancy, so parent node can be returned.
                } else {
                    QuadNodes.push_back(node->values.front());
                }

            }
            if(!node->values.empty() && node->values.front().occupancy != UNKNOWN && node->values.front().occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->values.empty())
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 920" << std::endl;
                    }
                }
            }
//            assert(false && "Coordinate not found in quadtree, something is going wrong");
        }

        void findAllIntersections(Node *node, std::vector<std::pair<QuadNode, QuadNode>> &intersections) const {
            // Find intersections between values stored in this node
            // Make sure to not report the same intersection twice
//            for (auto i = std::size_t(0); i < node->values.size(); ++i) {
//                for (auto j = std::size_t(0); j < i; ++j) {
//                    if (node->values[i].getBox().intersects(node->values[j].getBox()))
//                        intersections.emplace_back(node->values[i], node->values[j]);
//                }
//            }
//            if (!isLeaf(node)) {
//                // Values in this node can intersect values in descendants
//                for (const auto &child: node->children) {
//                    for (const auto &value: node->values)
//                        findIntersectionsInDescendants(child.get(), value, intersections);
//                }
//                // Find intersections in children
//                for (const auto &child: node->children)
//                    findAllIntersections(child.get(), intersections);
//            }
        }

        void
        findIntersectionsInDescendants(Node *node, const QuadNode &value,
                                       std::vector<std::pair<QuadNode, QuadNode>> &intersections) const {
            // Test against the values stored in this node
//            for (const auto &other: node->values) {
//                if (value.getBox().intersects(other.getBox()))
//                    intersections.emplace_back(value, other);
//            }
//            // Test against values stored into descendants of this node
//            if (!isLeaf(node)) {
//                for (const auto &child: node->children)
//                    findIntersectionsInDescendants(child.get(), value, intersections);
//            }
        }
    };

}
