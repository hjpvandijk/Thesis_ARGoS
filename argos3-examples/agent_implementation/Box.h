#pragma once

#include "coordinate.h"
#include <argos3/core/utility/logging/argos_log.h>

//Adapted from https://github.com/pvigier/Quadtree


namespace quadtree {

    class Box {
    public:
        double left;
        double top;
        double size; // Must be positive
//    double height; // Must be positive

        constexpr Box(double Left = 0, double Top = 0, double Size = 0) noexcept:
                left(Left), top(Top), size(Size) {

        }

        constexpr Box(const Coordinate position, double size) noexcept:
                left(position.x), top(position.y), size(size) {

        }

        constexpr double getRight() const noexcept {
            return left + size;
        }

        constexpr double getBottom() const noexcept {
            return top - size;
        }

        constexpr Coordinate getTopLeft() const noexcept {
            return {left, top};
        }

        constexpr Coordinate getCenter() const noexcept {
            return {left + size / 2, top - size / 2};
        }

        constexpr double getSize() const noexcept {
            return size;
        }

        [[nodiscard]] bool contains(const Box &box) const noexcept {
            bool result = left <= box.left && getRight() >= box.getRight() &&
                          top >= box.top && getBottom() <= box.getBottom();

//            if (!result) {
//                argos::LOG << "contains: " << left << " <= " << box.left << " && " << getRight() << " >= "
//                           << box.getRight()
//                           << " && "
//                           << top << " >= " << box.top << " && " << getBottom() << " <= " << box.getBottom()
//                           << std::endl;
//            }
            return result;

        }

        [[nodiscard]] bool contains(const Coordinate &coordinate) const noexcept {
            bool result = left <= coordinate.x && getRight() >= coordinate.x &&
                          top >= coordinate.y && getBottom() <= coordinate.y;

//            if (!result) {
//                argos::LOG << "contains: " << left << " <= " << coordinate.x << " && " << getRight() << " >= "
//                           << coordinate.x
//                           << " && "
//                           << top << " >= " << coordinate.y << " && " << getBottom() << " <= " << coordinate.y
//                           << std::endl;
//            }
            return result;

        }

        bool intersects_or_contains(const Box &box) const noexcept {

            //        return !(left >= box.getRight() || getRight() <= box.left ||
//            top >= box.getBottom() || getBottom() <= box.top);
//        return (left <= box.getRight() && getRight() >= box.left &&
//                 top <= box.getBottom() && getBottom() <= box.top);


            Coordinate topLeft = getTopLeft();
            Coordinate bottomRight = {getRight(), getBottom()};
            Coordinate topRight = {getRight(), top};
            Coordinate bottomLeft = {left, getBottom()};

            Coordinate boxTopLeft = box.getTopLeft();
            Coordinate boxBottomRight = {box.getRight(), box.getBottom()};
            Coordinate boxTopRight = {box.getRight(), box.top};
            Coordinate boxBottomLeft = {box.left, box.getBottom()};


            bool result = (box.contains(topLeft) || box.contains(bottomRight) || box.contains(topRight) ||
                     box.contains(bottomLeft) ||
                     contains(boxTopLeft) || contains(boxBottomRight) || contains(boxTopRight) ||
                     contains(boxBottomLeft));

//            if (!result){
//                argos::LOG << "querybox: " << topLeft.toString() << " " << bottomRight.toString() << " " << topRight.toString() << " " << bottomLeft.toString() << std::endl;
//                argos::LOG << "box: " << boxTopLeft.toString() << " " << boxBottomRight.toString() << " " << boxTopRight.toString() << " " << boxBottomLeft.toString() << std::endl;
//
//            }
            return result;
        }

        bool operator==(const Box &b) const {
            return this->left == b.left && this->top == b.top && this->size == b.size;
        }
    };

}