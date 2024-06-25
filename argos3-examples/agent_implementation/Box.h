#pragma once

#include "coordinate.h"
#include <argos3/core/utility/logging/argos_log.h>


namespace quadtree
{

class Box
{
public:
    double left;
    double top;
    double size; // Must be positive
//    double height; // Must be positive

    constexpr Box(double Left = 0, double Top = 0, double Size = 0) noexcept :
        left(Left), top(Top), size(Size)
    {

    }

    constexpr Box(const Coordinate position, double size) noexcept :
        left(position.x), top(position.y), size(size)
    {

    }

    constexpr double getRight() const noexcept
    {
        return left + size;
    }

    constexpr double getBottom() const noexcept
    {
        return top - size;
    }

    constexpr Coordinate getTopLeft() const noexcept
    {
        return {left, top};
    }

    constexpr Coordinate getCenter() const noexcept
    {
        return {left + size / 2, top - size / 2};
    }

    constexpr double getSize() const noexcept
    {
        return size;
    }

    [[nodiscard]] bool contains(const Box& box) const noexcept
    {
        bool result = left <= box.left && getRight() >= box.getRight() &&
               top >= box.top && getBottom() <= box.getBottom();

        if(!result) {
            argos::LOG << "contains: " << left << " <= " << box.left << " && " << getRight() << " >= " << box.getRight()
                       << " && "
                       << top << " >= " << box.top << " && " << getBottom() << " <= " << box.getBottom() << std::endl;
        }
        return result;

    }

    constexpr bool intersects(const Box& box) const noexcept
    {
        return !(left >= box.getRight() || getRight() <= box.left ||
            top >= box.getBottom() || getBottom() <= box.top);
    }

    bool operator==(const Box &b) const {
        return this->left == b.left && this->top == b.top && this->size == b.size;
    }
};

}