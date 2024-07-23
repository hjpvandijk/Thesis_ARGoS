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
        uint32_t visitedAtTicks;

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
        void add(Coordinate coordinate, Occupancy occupancy, uint32_t elapsed_ticks) {
            auto node = QuadNode{coordinate, occupancy, elapsed_ticks};
//            argos::LOG << "Adding node with elapsed ticks: " << node.visitedAtTicks << std::endl;
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
        std::vector<Box> queryOccupiedBoxes(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return queryBoxes(box, OCCUPIED);
        }

        /**
         * Returns all the unexplored boxes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<Box> queryUnexploredBoxes(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return queryBoxes(box, UNKNOWN);
        }

        /**
         * Returns all the frontier boxes surrounding the given coordinate within the given area size
         */
        std::vector<Box> queryFrontierBoxes(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            //A cell is a frontier iff (according to dynamic frontier-led swarming):
            //1. Occupancy = explored
            //2. At least one neighbor is unexplored using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

            //Get all the explored but free boxes
            std::vector<Box> exploredBoxes = queryBoxes(box, FREE);
//
//            //Get all the unexplored boxes
//            std::vector<Box> unexploredBoxes = queryBoxes(box, UNKNOWN);

            //Get all the frontier boxes
            std::vector<Box> frontierBoxes = {};
            for (Box exploredBox: exploredBoxes) {
                //Get the 8-connected Moore neighbours of the explored box
                bool isFrontier = false;
                if (isMooreNeighbourUnknown(exploredBox)) {
                    isFrontier = true;
                }
                if (isFrontier) {
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
//            std::vector<Occupancy> neighbours;
//            return true;
            //See if coordinate to the left is in the quadtree and get its occupancy
            Coordinate left = Coordinate{box.getCenter().x - box.size, box.getCenter().y};
//            argos::LOG << "LEFT: " << left.x << " , " << left.y << std::endl;
            if (mBox.contains(left)) {
//                neighbours.push_back({left, queryCoordinate(left).occupancy});
//                argos::LOG << "CONTAINED" << std::endl;
                if (getOccupancyFromCoordinate(left) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the right is in the quadtree and get its occupancy
            Coordinate right = Coordinate{box.getCenter().x + box.size, box.getCenter().y};
            if (mBox.contains(right)) {
//                neighbours.push_back({right, queryCoordinate(right).occupancy});
                if (getOccupancyFromCoordinate(right) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the top is in the quadtree and get its occupancy
            Coordinate top = Coordinate{box.getCenter().x, box.getCenter().y + box.size};
            if (mBox.contains(top)) {
//                neighbours.push_back({top, queryCoordinate(top).occupancy});
                if (getOccupancyFromCoordinate(top) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the bottom is in the quadtree and get its occupancy
            Coordinate bottom = Coordinate{box.getCenter().x, box.getCenter().y - box.size};
            if (mBox.contains(bottom)) {
//                neighbours.push_back({bottom, queryCoordinate(bottom).occupancy});
                if (getOccupancyFromCoordinate(bottom) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the top left is in the quadtree and get its occupancy
            Coordinate topLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y + box.size};
            if (mBox.contains(topLeft)) {
//                neighbours.push_back({topLeft, queryCoordinate(topLeft).occupancy});
                if (getOccupancyFromCoordinate(topLeft) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the top right is in the quadtree and get its occupancy
            Coordinate topRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y + box.size};
            if (mBox.contains(topRight)) {
//                neighbours.push_back({topRight, queryCoordinate(topRight).occupancy});
                if (getOccupancyFromCoordinate(topRight) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the bottom left is in the quadtree and get its occupancy
            Coordinate bottomLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y - box.size};
            if (mBox.contains(bottomLeft)) {
//                neighbours.push_back({bottomLeft, queryCoordinate(bottomLeft).occupancy});
                if (getOccupancyFromCoordinate(bottomLeft) == UNKNOWN) {
                    return true;
                }
            }

            //See if coordinate to the bottom right is in the quadtree and get its occupancy
            Coordinate bottomRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y - box.size};
            if (mBox.contains(bottomRight)) {
//                neighbours.push_back({bottomRight, queryCoordinate(bottomRight).occupancy});
                if (getOccupancyFromCoordinate(bottomRight) == UNKNOWN) {
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
            auto values = std::vector<QuadNode>();
            query(mRoot.get(), mBox, box, values, occupancy);
            return values;
        }

        /**
         * Returns all the values that intersect with or are contained by given box
         * @param box
         * @param occupancy
         * @return
         */
        std::vector<Box> queryBoxes(const Box &box, Occupancy occupancy) const {
            auto boxes = std::vector<Box>();
            queryBoxes(mRoot.get(), mBox, box, boxes, occupancy);
            return boxes;
        }

        /**
         * Returns the QuadNode containing the coordinate
         * @param coordinate
         */
        Occupancy getOccupancyFromCoordinate(Coordinate coordinate) const {
            auto values = std::vector<QuadNode>();
            return getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate).occupancy;
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

//                file << box.left << " " << box.top << " " << box.size << " " << "\n";

                // Write the bounding box, occupancy and depth of this node to the file
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                for (const auto &value: node->values) {
//                    file << box.left << " " << box.top << " " << box.size << " " << value.occupancy << " " << "\n";
                    QuadNode curQuadNode = value;
//                    argos::LOG << "Emplacing back: " << std::to_string(box.getCenter().x) << ';' << std::to_string(box.getCenter().y) << ':' << std::to_string(curQuadNode.occupancy) << std::endl;
                    strings->emplace_back(
                            std::to_string(box.getCenter().x) + ';' + std::to_string(box.getCenter().y) + ':' +
                            std::to_string(curQuadNode.occupancy));
                }

                // Traverse the children
                for (int i = 0; i < 4; ++i) {
                    if (node->children[i]) {
                        traverse(node->children[i].get(), computeBox(box, i), depth + 1);
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0);
        }

        /**
         * Get all the boxes in the quadtree
         * @return
         */
        std::vector<std::tuple<Box, int, uint32_t>> getAllBoxes() {
            std::vector<std::tuple<Box, int, uint32_t>> boxesAndOccupancyAndTicks = {};
            std::function<void(const Node *, const Box &, int, std::vector<std::tuple<Box, int, uint32_t>> *)> traverse;
            traverse = [&](const Node *node, const Box &box, int depth,
                           std::vector<std::tuple<Box, int, uint32_t>> *boxesAndOccupancyAndTicks) {
                if (node == nullptr) return;
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                for (const auto &value: node->values) {
                    boxesAndOccupancyAndTicks->emplace_back(std::tuple(box, value.occupancy, value.visitedAtTicks));
                }

                // Traverse the children
                for (int i = 0; i < 4; ++i) {
                    if (node->children[i]) {
                        traverse(node->children[i].get(), computeBox(box, i), depth + 1, boxesAndOccupancyAndTicks);
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0, &boxesAndOccupancyAndTicks);
            return boxesAndOccupancyAndTicks;

        }

        double getMinSize() {
            return this->MinSize;
        }

    private:
        static constexpr auto Threshold = std::size_t(16);
        static constexpr auto MaxDepth = std::size_t(8);
        static constexpr double MinSize = 0.2;

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

//            argos::LOG << "valueBox: " << valueBox.left << " , " << valueBox.getRight() << " , " << valueBox.getBottom()
//                       << " , " << valueBox.top << std::endl;
//            argos::LOG << "center: " << center.x << " , " << center.y << std::endl;
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

//            argos::LOG << "valueBox: " << valueBox.left << " , " << valueBox.getRight() << " , " << valueBox.getBottom()
//                       << " , " << valueBox.top << std::endl;
//            argos::LOG << "center: " << center.x << " , " << center.y << std::endl;
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
//            box.contains(value.getBox());
//            argos::LOG << "WITH BOX " << box.left << " , " << box.getRight() << " , " << box.getBottom() << " , "
//                       << box.top << std::endl;
            if (isLeaf(node)) {
//                argos::LOG << "Node is a leaf" << std::endl;
                // Insert the value in this node if possible

                //If the box size is the minimum size we allow (corresponding to finest mapping level), then we only contain a single QuadNode. Update the occupancy of this node to more important information.
                if (box.size <= MinSize) {
                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (node->values.empty()) {
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtTicks = value.visitedAtTicks;
                    } else {

                        if (node->values.front().occupancy == FREE || value.occupancy == FREE)
                            newNode.occupancy = FREE;
                        if (node->values.front().occupancy == OCCUPIED || value.occupancy == OCCUPIED)
                            newNode.occupancy = OCCUPIED;

                        newNode.visitedAtTicks = std::max(node->values.front().visitedAtTicks, value.visitedAtTicks);
//                        argos::LOG << "Setting elapsed ticks: " << newNode.visitedAtTicks << std::endl;

                    }
                    node->values.clear();
                    node->values.push_back(newNode);
                }
                    // Otherwise, we split and we try again
                else {
//                    argos::LOG << "Splitting node" << std::endl;
                    split(node, box);
                    add(node, depth, box, value);
                }
            } else {
//                argos::LOG << "Node is not a leaf" << std::endl;
                auto i = getQuadrant(box, value.coordinate);
//                argos::LOG << "Adding note to quadrant " << i << std::endl;
                // Add the value in a child if the value is entirely contained in it
                if (i != -1)
                    add(node->children[static_cast<std::size_t>(i)].get(), depth + 1, computeBox(box, i), value);
                    // Otherwise, we add the value in the current node
                else
                    node->values.push_back(value);
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
            for (auto &child: node->children)
                child = std::make_unique<Node>();
            // Assign values to children
            auto newValues = std::vector<QuadNode>(); // New values for this node
            for (const auto &value: node->values) {
                auto i = getQuadrant(box, value.coordinate);
//                argos::LOG << "Adding note to quadrant " << i << std::endl;
                if (i != -1)
                    node->children[static_cast<std::size_t>(i)]->values.push_back(value);
                else
                    newValues.push_back(value);
            }
            node->values = std::move(newValues);
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

            for (const auto &value: node->values) {
                if ((occupancy == ANY || value.occupancy == occupancy) &&
                    (queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
                    values.push_back(value);
            }
            if (!isLeaf(node)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
//                    argos::LOG << "NESTED" << std::endl;
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
                        Occupancy occupancy) const {
            assert(node != nullptr);
            assert(queryBox.intersects_or_contains(box));

            for (const auto &value: node->values) {
                if (value.occupancy == occupancy &&
                    (queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
                    boxes.push_back(box);
            }
            if (!isLeaf(node)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
//                    argos::LOG << "NESTED" << std::endl;
                    if (queryBox.intersects_or_contains(childBox))
                        queryBoxes(node->children[i].get(), childBox, queryBox, boxes, occupancy);
                }
            }
        }

        /**
         * @brief Get the QuadNode that contains the given coordinate
         * @param node
         * @param box
         * @param queryCoordinate
         * */
        QuadNode getQuadNodeFromCoordinate(Node *node, const Box &box, const Coordinate &queryCoordinate) const {
            assert(node != nullptr);
//            assert(queryBox.intersects_or_contains(box));
            assert(box.contains(queryCoordinate));

//            for (const auto &value: node->values) {
//                if ((queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
//                    boxes.push_back(box);
//            }
            //If it is a leaf node, return the QuadNode if it exists. If it does not exist, it means this coordinate is unexplored.
            if (isLeaf(node)) {
                if (node->values.size() == 0) {
                    return QuadNode{queryCoordinate, UNKNOWN, 0};
                } else {
                    return node->values.front();
                }
                // If it is not a leaf node, find the nested nodes, and search them.
            } else {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
//                    argos::LOG << "NESTED" << std::endl;
                    if (childBox.contains(queryCoordinate))
                        return getQuadNodeFromCoordinate(node->children[i].get(), childBox, queryCoordinate);
                }
            }
            assert(false && "Coordinate not found in quadtree, something is going wrong");
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
