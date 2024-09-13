#pragma once

#include <cassert>
#include <algorithm>
#include <array>
#include <memory>
#include <type_traits>
#include <vector>
#include "Box.h"
#include <iostream>

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

        template<class Archive>
        void serialize(Archive & archive)
        {
            archive(coordinate, occupancy, visitedAtS);
        }

    };

    class Quadtree {

    public:
        Quadtree(const Box &box) :
                mBox(box), mRoot(std::make_unique<Cell>()) {
            mRoot->quadNode = QuadNode{box.getCenter(), UNKNOWN, 0};
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

        void remove(const QuadNode &value) const{
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
        std::vector<Box> queryOccupiedBoxes(Coordinate coordinate, double areaSize, double currentTimeS) {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);

            return queryBoxes(box, OCCUPIED, currentTimeS);
        }

//        /**
//         * Returns all the unexplored boxes surrounding the given coordinate within the given area size
//         * @param coordinate
//         * @param areaSize
//         * @return
//         */
//        std::vector<Box> queryUnexploredBoxes(Coordinate coordinate, double areaSize, double currentTimeS) const {
//            // Create a box centered at the given coordinate
//            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);
//
//
//            return queryBoxes(box, UNKNOWN, currentTimeS);
//        }


void processBox(const Box& box, std::vector<Box>& frontierBoxes, int current_quadrant) const {
    if (box.size == getSmallestBoxSize()) {
        if (isMooreNeighbourUnknown(box, current_quadrant)) {
            frontierBoxes.push_back(box);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if(current_quadrant + i != 3){  //Only process the boxes that are at the outer edges of the queried box
                Box childBox = computeBox(box, i);
                processBox(childBox, frontierBoxes, i);
            }
        }

    }
}
/**
* Returns all the frontier boxes surrounding the given coordinate within the given area size
*/
std::vector<Box> queryFrontierBoxes(Coordinate coordinate, double areaSize, double currentTimeS) {
    Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);
    std::vector<Box> exploredBoxes = queryBoxes(box, FREE, currentTimeS);
    std::vector<Box> frontierBoxes;

    for (const Box& exploredBox : exploredBoxes) {
        processBox(exploredBox, frontierBoxes, -1);
    }

    return frontierBoxes;
}

        /**
         * Find if at least one of the 8-connected moore neighboring quadnodes of a given box is unexplored.
         * @param box
         * @return
         */
        bool isMooreNeighbourUnknown(const Box &box, int current_quadrant) const {
//            argos::LOG << "Checking moore neighbours of box: " << box.getCenter().x << " " << box.getCenter().y << " of size " << box.getSize() << std::endl;
            //0=NW, 1=NE, 2=SW, 3=SE

            //Only check the moore neighbours that are at the outer edges of the queried box
            //So only check if current quadrant in the WEST (left)
            if(current_quadrant==-1 || current_quadrant==0 || current_quadrant==2) {
                //See if coordinate to the left is in the quadtree and get its occupancy
                Coordinate left = Coordinate{box.getCenter().x - box.size, box.getCenter().y};
                if (mBox.contains(left)) {
//                argos::LOG << "left: " << left.x << " " << left.y << std::endl;
                    if (isCoordinateUnknown(left)) {
                        return true;
                    }
                }
            }
            //So only check if current quadrant in the EAST (right)
            if(current_quadrant==-1 || current_quadrant==1 || current_quadrant==3) {
                //See if coordinate to the right is in the quadtree and get its occupancy
                Coordinate right = Coordinate{box.getCenter().x + box.size, box.getCenter().y};
                if (mBox.contains(right)) {
//                argos::LOG << "right: " << right.x << " " << right.y << std::endl;
                    if (isCoordinateUnknown(right)) {
                        return true;
                    }
                }
            }
            //So only check if current quadrant in the NORTH (top)
            if(current_quadrant==-1 || current_quadrant==0 || current_quadrant==1) {
                //See if coordinate to the top is in the quadtree and get its occupancy
                Coordinate top = Coordinate{box.getCenter().x, box.getCenter().y + box.size};
                if (mBox.contains(top)) {
//                argos::LOG << "top: " << top.x << " " << top.y << std::endl;
                    if (isCoordinateUnknown(top)) {
                        return true;
                    }
                }
            }
            //So only check if current quadrant in the SOUTH (bottom)
            if(current_quadrant==-1 || current_quadrant==2 || current_quadrant==3) {
                //See if coordinate to the bottom is in the quadtree and get its occupancy
                Coordinate bottom = Coordinate{box.getCenter().x, box.getCenter().y - box.size};
                if (mBox.contains(bottom)) {
//                argos::LOG << "bottom: " << bottom.x << " " << bottom.y << std::endl;
                    if (isCoordinateUnknown(bottom)) {
                        return true;
                    }
                }
            }
            //So don't check if current quadrant in the SOUTH EAST (bottom right)
            if(current_quadrant==-1 || current_quadrant==0 || current_quadrant==1 || current_quadrant==2) {
                //See if coordinate to the top left is in the quadtree and get its occupancy
                Coordinate topLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y + box.size};
                if (mBox.contains(topLeft)) {
//                argos::LOG << "topLeft: " << topLeft.x << " " << topLeft.y << std::endl;
                    if (isCoordinateUnknown(topLeft)) {
                        return true;
                    }
                }
            }
            //So don't check if current quadrant in the SOUTH WEST (bottom left)
            if(current_quadrant==-1 || current_quadrant==0 || current_quadrant==1 || current_quadrant==3) {
                //See if coordinate to the top right is in the quadtree and get its occupancy
                Coordinate topRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y + box.size};
                if (mBox.contains(topRight)) {
//                argos::LOG << "topRight: " << topRight.x << " " << topRight.y << std::endl;
                    if (isCoordinateUnknown(topRight)) {
                        return true;
                    }
                }
            }
            //So don't check if current quadrant in the NORTH EAST (top right)
            if(current_quadrant==-1 || current_quadrant==0 || current_quadrant==2 || current_quadrant==3) {
                //See if coordinate to the bottom left is in the quadtree and get its occupancy
                Coordinate bottomLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y - box.size};
                if (mBox.contains(bottomLeft)) {
//                argos::LOG << "bottomLeft: " << bottomLeft.x << " " << bottomLeft.y << std::endl;
                    if (isCoordinateUnknown(bottomLeft)) {
                        return true;
                    }
                }
            }
            //So don't check if current quadrant in the NORTH WEST (top left)
            if(current_quadrant==-1 || current_quadrant==1 || current_quadrant==2 || current_quadrant==3) {
                //See if coordinate to the bottom right is in the quadtree and get its occupancy
                Coordinate bottomRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y - box.size};
                if (mBox.contains(bottomRight)) {
//                argos::LOG << "bottomRight: " << bottomRight.x << " " << bottomRight.y << std::endl;
                    if (isCoordinateUnknown(bottomRight)) {
                        return true;
                    }
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
        std::vector<Box> queryBoxes(const Box &box, Occupancy occupancy, double currentTimeS) {
            auto boxes = std::vector<Box>();
            //While querying we also check if pheromones are expired, we remember those and remove them after.
            std::vector<QuadNode> values_to_be_removed = {};
            queryBoxes(mRoot.get(), mBox, box, boxes, occupancy, currentTimeS, values_to_be_removed);
            for(auto &value: values_to_be_removed) {
                remove(value);
            }
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
            assert(QuadNodes.size() == 1);
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

            std::function<void(const Cell *, const Box &, int)> traverse;
            traverse = [&](const Cell *cell, const Box &box, int depth) {
                if (cell == nullptr) return;

                file << box.left << " " << box.top << " " << box.size << " " << "\n";

                // Write the bounding box, occupancy and depth of this cell to the file
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                file << box.left << " " << box.top << " " << box.size << " " << cell->quadNode.occupancy << " " << "\n";

                // Traverse the children
                for (int i = 0; i < 4; ++i) {
                    if (cell->children[i]) {
                        traverse(cell->children[i].get(), computeBox(box, i), depth + 1);
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
            std::function<void(const Cell *, const Box &, int, int &, std::string &)> traverse;
            std::string grouped_message = "";
            grouped_message.clear();
            int counter = 0;

            traverse = [&](const Cell *cell, const Box &box, int depth, int &counter, std::string &grouped_message) {
                if (cell == nullptr) return;

                bool allSameOccupancy = false;

                // Write the bounding box, occupancy and depth of this cell to the file
                    // If the occupancy is ANY, we don't need to store it, as the children will have new info
                    // If the occupancy is UNKNOWN, we don't need to store it, as a child not existing will also yield in an UNKNOWN
//                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
//                        continue;
                    // If the occupancy is OCCUPIED or FREE, we want to exchange that information. And we don't have to send any children as they will be all the same.
                    if (cell->quadNode.occupancy == OCCUPIED || cell->quadNode.occupancy == FREE){
                        allSameOccupancy = true;

                        std::string str =
                                std::to_string(box.getCenter().x) + ';' + std::to_string(box.getCenter().y) + ':' +
                                std::to_string(cell->quadNode.occupancy) + '@' + std::to_string(cell->quadNode.visitedAtS);

                        //Group every 10 nodes
                        grouped_message.append(str);

                        if(counter==49){
                            strings->emplace_back(grouped_message);
                            grouped_message.clear();
                            counter = 0;
                        }
                        else {
                            grouped_message.append("|");
                            counter++;
                        }

                    }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    // Traverse the children
                    for (int i = 0; i < 4; i++) {
                        if (cell->children[i]) {
                            traverse(cell->children[i].get(), computeBox(box, i), depth + 1, counter, grouped_message);
                        }
                    }
                }
            };

            traverse(mRoot.get(), mBox, 0, counter, grouped_message);
            //If there is an incomplete group, also send it.
            if(!grouped_message.empty()){
                grouped_message.pop_back(); //Delete the last delimiter
                strings->emplace_back(grouped_message);
            }
        }

        /**
         * Get all the boxes in the quadtree
         * @return
         */
        std::vector<std::tuple<Box, int, double>> getAllBoxes() {
            std::vector<std::tuple<Box, int, double>> boxesAndOccupancyAndTicks = {};
            std::function<void(const Cell *, const Box &, int, std::vector<std::tuple<Box, int, double>> *)> traverse;
            traverse = [&](const Cell *cell, const Box &box, int depth,
                           std::vector<std::tuple<Box, int, double>> *boxesAndOccupancyAndTicks) {
                if (cell == nullptr) return;
                bool allSameOccupancy = false;
//                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
//                        continue;
                    // If the occupancy is OCCUPIED or FREE, we want to exchange that information. And we don't have to send any children as they will be all the same.
                    if (cell->quadNode.occupancy == OCCUPIED || cell->quadNode.occupancy == FREE) {
                        allSameOccupancy = true;
                        boxesAndOccupancyAndTicks->emplace_back(std::tuple(box, cell->quadNode.occupancy, cell->quadNode.visitedAtS));
                   }

                if(cell->quadNode.visitedAtS != -1 && cell->quadNode.occupancy != UNKNOWN && cell->quadNode.occupancy != ANY){
                    if(!isLeaf(cell)) {
                        for(const auto & i : cell->children) {
                            if(i->quadNode.visitedAtS == -1)
                                argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line GetAllBoxes" << std::endl;
                        }
                    }
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    for (int i = 0; i < 4; i++) {
                        if (cell->children[i]) {
                            traverse(cell->children[i].get(), computeBox(box, i), depth + 1, boxesAndOccupancyAndTicks);
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

        double getSmallestBoxSize() const {
            return this->Smallest_Box_Size;
        }


    private:
        static constexpr auto Threshold = std::size_t(16);
        static constexpr double MinSize = 0.2;
        double Smallest_Box_Size = MinSize;
        static constexpr double EvaporationTime = 100.0;
        static constexpr double MaxAllowedVisitedTimeDiffS = 10.0;


        struct Cell {
            std::array<std::unique_ptr<Cell>, 4> children;
//            std::vector<QuadNode> values;
            QuadNode quadNode = QuadNode{Coordinate{0,0}, Occupancy::UNKNOWN,-1};
        };

        Box mBox;
        std::unique_ptr<Cell> mRoot;

        /**
         * @brief Check if the given node is a leaf i.e. had no children
         * @param node
         * @return
         */
        bool isLeaf(const Cell *node) const {
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
         * @param cell
         * @param depth
         * @param box
         * @param value
         */
        void add(Cell *cell, std::size_t depth, const Box &box, const QuadNode &value) {
            assert(cell != nullptr);
            assert(box.contains(value.coordinate));

            assert(value.occupancy != ANY && "Added occupancy should not be ANY");
            assert(value.occupancy != UNKNOWN && "Added occupancy should not be UNKNOWN");
            if (isLeaf(cell)) {
                // Insert the value in this cell if possible

                //If the box size is the minimum size we allow (corresponding to finest mapping level),
                // then we only contain a single QuadNode. Update the occupancy of this cell to the most important occupancy.
                if (box.size <= MinSize) {
                    if(box.size <= Smallest_Box_Size) Smallest_Box_Size = box.size;

                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (cell->quadNode.visitedAtS == -1) { //If the cell is empty
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                    } else {
                        //OCCUPIED always takes precedence over FREE
                        //Update with the most precedent or up-to-date information.
                        if (cell->quadNode.occupancy == OCCUPIED && value.occupancy == OCCUPIED) {
                            newNode.occupancy = OCCUPIED;
                            newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
                        } else if (cell->quadNode.occupancy == OCCUPIED){
                            newNode.occupancy = OCCUPIED;
                            newNode.visitedAtS = cell->quadNode.visitedAtS;
                        } else if (value.occupancy == OCCUPIED){
                            newNode.occupancy = OCCUPIED;
                            newNode.visitedAtS = value.visitedAtS;
                        } else if (cell->quadNode.occupancy == FREE && value.occupancy == FREE) {
                            newNode.occupancy = FREE;
                            newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
                        } else if (cell->quadNode.occupancy == FREE){
                            newNode.occupancy = FREE;
                            newNode.visitedAtS = cell->quadNode.visitedAtS;
                        } else if (value.occupancy == FREE){
                            newNode.occupancy = FREE;
                            newNode.visitedAtS = value.visitedAtS;
                        } else {
                            assert(-1 && "Shouldn't get here, as neither current cell or added cell are OCCUPIED or FREE");
                        }


                    }
                    assert(newNode.occupancy == FREE || newNode.occupancy == OCCUPIED && "new cell occupancy should be FREE or OCCUPIED");
                    // Make the only value the 'merged cell'
                    cell->quadNode = newNode;
                }
                    // Otherwise, we split and we try again
                else {
                    //If the to be added occupancy is the same as the parent, and the visited time is not too far apart, we can skip adding.
                    if (cell->quadNode.visitedAtS == -1 || !(value.occupancy == cell->quadNode.occupancy && value.visitedAtS - cell->quadNode.visitedAtS <= MaxAllowedVisitedTimeDiffS)) {
                        split(cell, box);
                        add(cell, depth, box, value);
                    }
                }
            } else {
                // If the cell is not a leaf
                // And if the box center is the same as the value coordinate (meaning this value information is the same for all children of this cell),
                // then we only contain a single QuadNode. Update the occupancy of this cell to the most important occupancy.
                // This should only happen when adding nodes received from other agents.
                if (box.getCenter() == value.coordinate) {

//                    argos::LOG << " CENTER OF THE BOX" << std::endl;
                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (cell->quadNode.visitedAtS == -1) {
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                    } else {
                        //If cell has occupancy FREE or OCCUPIED, it entails all its children are also of that value, so we just update this parent.
                        if(cell->quadNode.occupancy == FREE){
                            if(value.occupancy == OCCUPIED) {
                                newNode.occupancy = OCCUPIED;
                                newNode.visitedAtS = value.visitedAtS;
                            } else {
                                newNode.occupancy = FREE;
                                newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
                            }
                            cell->quadNode = newNode;
                        } else if (cell->quadNode.occupancy == OCCUPIED){
                            if(value.occupancy == FREE) {
                                newNode.occupancy = OCCUPIED;
                                newNode.visitedAtS = cell->quadNode.visitedAtS;
                            } else {
                                newNode.occupancy = OCCUPIED;
                                newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
                            }
                            cell->quadNode = newNode;
                        //Else if cell has occupancy ANY or UNKNOWN, we add to the children
                        } else {
                            //When adding a new coordinate and occupancy to that parent, we should set the remaining (yet unset) children to the value occupancy.
                            //What will happen:
                            //Children without a value will be set with information entailed in the received value
                            //Children with a value will compare the occupancy and visited times for precedence and most recent and update accordingly.
                            //If after updating a child, all children have the same occupancy and visited times are within range, the parent will entail the details and the children will be deleted.
                            for (int i = 0; i < cell->children.size(); i++) {
                                Box childBox = computeBox(box, i);
                                Coordinate childBoxCenter = childBox.getCenter();

                                // For each child, create a cell with corresponding center coordinate
                                QuadNode newChildNode = QuadNode();
                                newChildNode.visitedAtS = value.visitedAtS;
                                newChildNode.coordinate = childBoxCenter;
                                newChildNode.occupancy = value.occupancy;

                                //Add to current cell, so that it will be placed in the proper child and checked for optimization later.
                                add(cell, depth, box, newChildNode);

                            }
                        }

                    }
//
                // Else we add the value to the appropriate child
                } else {
                    auto i = getQuadrant(box, value.coordinate);
                    // Add the value in a child if the value is entirely contained in it
                    assert(i != -1 && "A value should be contained in a quadrant");
                    assert(i != 4 && "A value should not be the same as the center of the box");
                    add(cell->children[static_cast<std::size_t>(i)].get(), depth + 1, computeBox(box, i), value);

//                Check if all children have the same occupancy
                    Occupancy firstOccupancy = UNKNOWN;
                    bool allSameOccupancy = true;
                    bool visitedTimesTooFarApart = false;
                    double minVisitedTime = MAXFLOAT;
                    double maxVisitedTime = -1;

                    if (cell->children[0]->quadNode.visitedAtS == -1)
                        allSameOccupancy = false;
                    else {
                        //Get the occupancy of the first child (if it is empty, it is not the same occupancy as the others)
                        firstOccupancy = cell->children[0]->quadNode.occupancy;
                        for (const auto &child: cell->children) {
                            //If a child is empty, it is not the same occupancy as the others
                            //If a child has a different occupancy than the first child, it is not the same occupancy as all others
                            //If a child has occupancy ANY, further nested nodes will have a different occupancy.
                            if (child->quadNode.visitedAtS == -1 || child->quadNode.occupancy == ANY ||
                                child->quadNode.occupancy != firstOccupancy) {
                                allSameOccupancy = false;
                                break;
                            }
                        //If the visited time of the child is too far apart from the first child, it is not the same occupancy as the others
                            if(child->quadNode.visitedAtS < minVisitedTime)
                                minVisitedTime = child->quadNode.visitedAtS;
                            if(child->quadNode.visitedAtS > maxVisitedTime)
                                maxVisitedTime = child->quadNode.visitedAtS;
                        }
                        //If the visited times are too far apart, the children should be kept
                        if (maxVisitedTime - minVisitedTime > MaxAllowedVisitedTimeDiffS)
                            visitedTimesTooFarApart = true;
                    }
//                assert(!cell->quadNode.visitedAtS == -1 && "A non-leaf cell should have a value");

                    // If all children have the same occupancy, and their visited times are not too far apart, we can delete the children and the parents will have all info
                    if(allSameOccupancy && !visitedTimesTooFarApart) {
                        //Unitialize the children nodes as the parent now contains their information.
                        for (auto &child: cell->children)
                            child.reset();
                        assert(isLeaf(cell) && "The cell should be a leaf again now");

                        //If all children have the same occupancy, give the parent that occupancy
                        cell->quadNode.occupancy = firstOccupancy;
                        assert(cell->quadNode.occupancy != UNKNOWN && "A non-leaf cell should not have UNKNOWN occupancy");
//                        argos::LOG << "Parent cell occupancy: " << cell->quadNode.occupancy << std::endl;
                        cell->quadNode.visitedAtS = maxVisitedTime;

                    } else {
                        //If the children have different occupancies, or the visited times are too far apart, the parent should have occupancy ANY
                        cell->quadNode.occupancy = ANY;

                    }
                }

            }
            if(cell->quadNode.visitedAtS != -1 && cell->quadNode.occupancy != UNKNOWN && cell->quadNode.occupancy != ANY){
                if(!isLeaf(cell)) {
                    for(const auto & i : cell->children) {
                        if(i->quadNode.visitedAtS == -1)
                            argos::LOGERR << "leaf value empty while parent is " << cell->quadNode.occupancy << " 651" << std::endl;
                    }
                }
            }
        }

        /**
         * @brief Split a leaf cell into four children
         * @param cell
         * @param box
         */
        void split(Cell *cell, const Box &box) {
            assert(cell != nullptr);
            assert(isLeaf(cell) && "Only leaves can be split");
            // Create children
//            argos::LOG << cell->children.size() << std::endl;
//            argos::LOG << cell->children[0]->children.size() << std::endl;
//            auto newValues = std::vector<QuadNode>(); // New values for this cell



            for (auto &child: cell->children)
                child = std::make_unique<Cell>();

            assert(!isLeaf(cell) && "A cell should not be a leaf after splitting");

            //If cell has occupancy FREE or OCCUPIED, it entails all its children are also of that value.
            //When adding a new coordinate and occupancy to that parent, we should set the three remaining children to the parent occupancy.
            //So when splitting we set all children to the parent occupancy and visited time.
            if(cell->quadNode.visitedAtS != -1 && (cell->quadNode.occupancy == FREE || cell->quadNode.occupancy == OCCUPIED)) {
                for (int i = 0; i < cell->children.size(); i++) {
                    Box childBox = computeBox(box, i);
                    Coordinate childBoxCenter = childBox.getCenter();

                    auto quadNode = QuadNode{childBoxCenter, cell->quadNode.occupancy,
                                             cell->quadNode.visitedAtS};
                    add(cell->children[static_cast<std::size_t>(i)].get(), 6, childBox, quadNode);
                }
            }
            cell->quadNode = QuadNode{box.getCenter(), ANY, 0};
        }
        /**
         * @brief Remove a value from the quadtree
         * @param node
         * @param box
         * @param value
         * @return
         */
        void remove(Cell *node, const Box &box, const QuadNode &value) const {
            assert(node != nullptr);
            assert(box.contains(value.coordinate));
            if (isLeaf(node)) {
                // Remove the value from node
                removeValue(node, value);
            } else {
                // Remove the value in a child if the value is entirely contained in it
                auto i = getQuadrant(box, value.coordinate);
                if (i ==
                    4) { // If the value is the same as the center of the box, we remove the value from the current node
                    removeValue(node, value);
                    for (auto &child: node->children)
                        child.reset();
                } else {
                    assert(i != -1);
                    remove(node->children[static_cast<std::size_t>(i)].get(), computeBox(box, i), value);
//                    node->children[static_cast<std::size_t>(i)].reset();
                    node->quadNode.occupancy = ANY;
                    // Check if there are any children, if not, remove the value from the parent
                    bool atLeastOneChildWithValue = false;
                    for (auto &child: node->children)
                        if (child->quadNode.visitedAtS != -1) atLeastOneChildWithValue = true;

                    if (!atLeastOneChildWithValue) {
                        removeValue(node, node->quadNode);
                        for (auto &child: node->children)
                            child.reset();

                        assert(isLeaf(node));
                    }

                }
            }
        }

        void removeValue(Cell *node, const QuadNode &value) const{
            assert(node->quadNode == value); //We can only remove a value from a node which has that value
            node->quadNode = QuadNode{Coordinate{0,0}, UNKNOWN, -1}; //visitedAtS -1 means it is empty
        }

        /**
         * @brief Query the quadtree for QuadNodes that intersect with or are contained by the given box
         * @param node
         * @param box
         * @param queryBox
         * @param values
         * @param occupancy
         */
        void query(Cell *node, const Box &box, const Box &queryBox, std::vector<QuadNode> &values,
                   Occupancy occupancy) const {
            assert(node != nullptr);
            assert(queryBox.intersects_or_contains(box));
            if ((occupancy == ANY || node->quadNode.occupancy == occupancy) &&
                (queryBox.contains(node->quadNode.coordinate) || queryBox.intersects_or_contains(box)))
                values.push_back(node->quadNode);
            if(node->quadNode.visitedAtS != -1 && node->quadNode.occupancy != UNKNOWN && node->quadNode.occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->quadNode.visitedAtS == -1)
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 771" << std::endl;
                    }
                }
            }
            //Only check further if the occupancy of the non-leaf node is not all the same for its children, so UNKNOWN.
            if (!isLeaf(node) && (node->quadNode.visitedAtS == -1 || node->quadNode.occupancy==ANY || node->quadNode.occupancy==UNKNOWN)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
                    if (queryBox.intersects_or_contains(childBox))
                        query(node->children[i].get(), childBox, queryBox, values, occupancy);
                }
            }
        }
        /**
         * @brief Query the quadtree for boxes that intersect with or are contained by the given box that have a given occupancy
         * @param cell the current cell being looked in
         * @param box the box the current cell belongs in
         * @param queryBox the search space
         * @param boxes the list of boxes in the search space with the correct occupancy
         * @param occupancy the occupancy to look for
         * @param currentTimeS the current experiment time of the agent
         * @param values_to_be_removed the list of values that need to be removed from the quadtree as they are expired
         */
        void queryBoxes(Cell *cell, const Box &box, const Box &queryBox, std::vector<Box> &boxes,
                        Occupancy occupancy, double currentTimeS, std::vector<QuadNode> &values_to_be_removed) {
            assert(cell != nullptr);
            assert(queryBox.intersects_or_contains(box));
            //Check if pheromone is expired, if so, set the occupancy to unknown and remove it later
            if(cell->quadNode.occupancy==Occupancy::FREE && calculatePheromone(cell->quadNode.visitedAtS, currentTimeS)<0.05){
                cell->quadNode.occupancy = Occupancy::UNKNOWN;
                //Keep a list of the to be removed values, as we cant delete them now due to concurrency issues. These will be deleted after the querying is done.
                values_to_be_removed.push_back(cell->quadNode);
            }


            if (cell->quadNode.occupancy == occupancy &&
                (queryBox.contains(cell->quadNode.coordinate) || queryBox.intersects_or_contains(box)))
                boxes.push_back(box);

            if(!cell->quadNode.visitedAtS == -1 && cell->quadNode.occupancy != UNKNOWN && cell->quadNode.occupancy != ANY){
                if(!isLeaf(cell)) {
                    for(const auto & i : cell->children) {
                        if(i->quadNode.visitedAtS == -1)
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 813" << std::endl;
                    }
                }
            }


            //Only check further if the occupancy of the non-leaf cell is not all the same for its children, so ANY.
            if (!isLeaf(cell) && (cell->quadNode.visitedAtS == -1 || cell->quadNode.occupancy == ANY || cell->quadNode.occupancy == UNKNOWN)) {
                for (int d = 0; d < cell->children.size(); d++) {
                    auto childBox = computeBox(box, static_cast<int>(d));
                    if (queryBox.intersects_or_contains(childBox)) {
                        queryBoxes(cell->children[d].get(), childBox, queryBox, boxes, occupancy, currentTimeS, values_to_be_removed);
                    }
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
        void getQuadNodesFromCoordinate(Cell *node, const Box &box, const Coordinate &queryCoordinate, std::vector<QuadNode> & QuadNodes) const {
            assert(node != nullptr);
            assert(box.contains(queryCoordinate));
            //If it is a leaf node, return the QuadNode if it exists. If it does not exist, it means this coordinate is unexplored.
            if (isLeaf(node)) {
                if (node->quadNode.visitedAtS == -1) {
//                    argos::LOG << "Yes it is because of this" << std::endl;
                    QuadNodes.push_back(QuadNode{queryCoordinate, UNKNOWN, 0});
                } else {
                    assert(node->quadNode.occupancy != ANY && "leaf occupancy should never be ANY");
                    QuadNodes.push_back(node->quadNode);
                }
                // If it is not a leaf node, find the nested nodes, and search them.
            } else {
                //If the node occupancy is ANY or UNKNOWN, there can be nested nodes with different occupancies
                assert(!node->quadNode.visitedAtS == -1 && "Cell should have a value");
                if(node->quadNode.occupancy ==ANY || node->quadNode.occupancy == UNKNOWN) {
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
                    QuadNodes.push_back(node->quadNode);
                }

            }
            if(!node->quadNode.visitedAtS == -1 && node->quadNode.occupancy != UNKNOWN && node->quadNode.occupancy != ANY){
                if(!isLeaf(node)) {
                    for(const auto & i : node->children) {
                        if(i->quadNode.visitedAtS == -1)
                            argos::LOGERR << "leaf value empty while parent Isn't ANY or UNKNOWN line 920" << std::endl;
                    }
                }
            }
//            assert(false && "Coordinate not found in quadtree, something is going wrong");
        }

        void findAllIntersections(Cell *node, std::vector<std::pair<QuadNode, QuadNode>> &intersections) const {
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
        findIntersectionsInDescendants(Cell *node, const QuadNode &value,
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
