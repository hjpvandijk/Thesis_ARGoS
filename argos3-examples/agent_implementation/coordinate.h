//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_COORDINATE_H
#define THESIS_ARGOS_COORDINATE_H

#include <string>


struct coordinate {
    double x;
    double y;
//
//    coordinate() {}
//
//    coordinate(double x, double y);
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


};



struct Vector2 {
    double x;
    double y;
};

#endif //THESIS_ARGOS_COORDINATE_H
