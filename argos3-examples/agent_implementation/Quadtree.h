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
    };


    struct QuadNode {
        Coordinate coordinate;
        Occupancy occupancy;

        bool operator==(const QuadNode &rhs) const {
            return coordinate.x == rhs.coordinate.x && coordinate.y == rhs.coordinate.y;
        }
    };

    class Quadtree {

    public:
        Quadtree(const Box &box) :
                mBox(box), mRoot(std::make_unique<Node>()) {
            argos::LOG << "CREATED QUADTREE WITH BOX " << box.left << " , " << box.getRight() << " , "
                       << box.getBottom() << " , " << box.top << std::endl;

        }

        void add(Coordinate coordinate, Occupancy occupancy) {
            auto node = QuadNode{coordinate, occupancy};
            add(node);
        }

        void add(const QuadNode &value) {
            add(mRoot.get(), 0, mBox, value);
        }

        void remove(const QuadNode &value) {
            remove(mRoot.get(), mBox, value);
        }

        // Returns all the values surrounding the given coordinate within the given area size
        std::vector<QuadNode> queryOccupied(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x-areaSize/2.0, coordinate.y+areaSize/2.0}, areaSize);


            return query(box, OCCUPIED);
        }

        // Returns all the values that intersect with the given box
        std::vector<QuadNode> query(const Box &box, Occupancy occupancy) const {
            auto values = std::vector<QuadNode>();
            query(mRoot.get(), mBox, box, values, occupancy);
            return values;
        }

        std::vector<std::pair<QuadNode, QuadNode>> findAllIntersections() const {
            auto intersections = std::vector<std::pair<QuadNode, QuadNode>>();
            findAllIntersections(mRoot.get(), intersections);
            return intersections;
        }

        Box getBox() const {
            return mBox;
        }

        void exportQuadtreeToFile(const std::string& filename) {
            std::ofstream file(filename + ".txt");
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return;
            }

            std::function<void(const Node*, const Box&, int)> traverse;
            traverse = [&](const Node* node, const Box& box, int depth) {
                if (node == nullptr) return;

                file << box.left << " " << box.top << " " << box.size << " " << "\n";

                // Write the bounding box, occupancy and depth of this node to the file
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                for (const auto& value : node->values) {
                    file << box.left << " " << box.top << " " <<  box.size << " " << value.occupancy << " " << "\n";
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

        double getMinSize(){
            return this->MinSize;
        }

    private:
        static constexpr auto Threshold = std::size_t(16);
        static constexpr auto MaxDepth = std::size_t(8);
        static constexpr double MinSize = 0.5;

        struct Node {
            std::array<std::unique_ptr<Node>, 4> children;
            std::vector<QuadNode> values;
        };

        Box mBox;
        std::unique_ptr<Node> mRoot;

        bool isLeaf(const Node *node) const {
            return !static_cast<bool>(node->children[0]);
        }

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

        int getQuadrant(const Box &nodeBox, const Coordinate& valueCoordinate) const {
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

        void add(Node *node, std::size_t depth, const Box &box, const QuadNode &value) {
            assert(node != nullptr);
            assert(box.contains(value.coordinate));
//            box.contains(value.getBox());
//            argos::LOG << "WITH BOX " << box.left << " , " << box.getRight() << " , " << box.getBottom() << " , "
//                       << box.top << std::endl;
            if (isLeaf(node)) {
//                argos::LOG << "Node is a leaf" << std::endl;
                // Insert the value in this node if possible

                //If the box size is the minimum size we allow (corresponding to finest mapping level), then we only contain a single QuadNode.
                if (box.size <= MinSize) {
                    node->values.clear();
                    node->values.push_back(value);
                    //Else we can add more QuadNodes if there is space.
                }
//                else if (node->values.size() < Threshold) {
//                    argos::LOG << "Adding value to node" << std::endl;
//                    node->values.push_back(value);
//                }
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

        void query(Node *node, const Box &box, const Box &queryBox, std::vector<QuadNode> &values, Occupancy occupancy) const {
            assert(node != nullptr);
//            assert(queryBox.intersects(box)||box.contains(queryBox) && "Query box must intersect or contain the node box");
//            bool assert = queryBox.intersects(box)||box.contains(queryBox);
            assert(queryBox.intersects_or_contains(box));
//            assert(queryBox.contains(box) && "Query box must contain the node box");

            for (const auto &value: node->values) {
                if (value.occupancy == occupancy && (queryBox.contains(value.coordinate) || queryBox.intersects_or_contains(box)))
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
