//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_COORDINATE_H
#define THESIS_ARGOS_COORDINATE_H

#include <string>
#include <argos3/core/utility/math/angles.h>


struct Coordinate {
    double x;
    double y;
//
//    Coordinate() {}
//
//    Coordinate(double x, double y);
//
//    void setCoordinates(double new_x, double new_y);
//
//    void setX(double new_x);
//
//    void setY(double new_y);
//
//    double getX();
//
//    double getY();

//    std::string toString();



    std::string toString() {
        return std::to_string(this->x) + ";" + std::to_string(this->y);
    }

    /**
     * @brief Converts a heading from the own coordinate system to the argos coordinate system
     * @param radians
     * @return
     */
    static argos::CRadians OwnHeadingToArgos(argos::CRadians radians) {
        return argos::CRadians(radians - argos::CRadians::PI_OVER_TWO);
    }

    /**
     * @brief Converts a heading from the argos coordinate system to the own coordinate system
     * @param radians
     * @return
     */
    static argos::CRadians ArgosHeadingToOwn(argos::CRadians radians) {
        return argos::CRadians(radians + argos::CRadians::PI_OVER_TWO);
    }

    /**
     * @brief Converts the coordinate from the own coordinate system to the argos coordinate system
     * @return
     */
    Coordinate FromOwnToArgos() {
        return Coordinate{y, -x};
    }

    /**
     * @brief Converts the coordinate from the argos coordinate system to the own coordinate system
     * @return
     */
    Coordinate FromArgosToOwn() {
        return Coordinate{-y, x};
    }

    /**
     * @brief Compares two coordinates for equality
     * @return
     */
    bool operator==(const Coordinate &rhs) const {
        return x == rhs.x &&
               y == rhs.y;
    }



};



struct Vector2 {
    double x;
    double y;
};

#endif //THESIS_ARGOS_COORDINATE_H
