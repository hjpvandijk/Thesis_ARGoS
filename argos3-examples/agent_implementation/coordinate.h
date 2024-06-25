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

    static argos::CRadians OwnHeadingToArgos(argos::CRadians radians) {
        return argos::CRadians(radians - argos::CRadians::PI_OVER_TWO);
    }

    static argos::CRadians ArgosHeadingToOwn(argos::CRadians radians) {
        return argos::CRadians(radians + argos::CRadians::PI_OVER_TWO);
    }

    Coordinate FromOwnToArgos() {
        return Coordinate{y, -x};
    }

    Coordinate FromArgosToOwn() {
        return Coordinate{-y, x};
    }



};



struct Vector2 {
    double x;
    double y;
};

#endif //THESIS_ARGOS_COORDINATE_H
