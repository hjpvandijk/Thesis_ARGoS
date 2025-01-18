//
// Created by hugo on 16-10-24.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_HC_SR04_H
#define IMPLEMENTATION_AND_EXAMPLES_HC_SR04_H


class HC_SR04 {
public:
    float distance;

    HC_SR04() : distance(0) {}

    float getDistance() const {
        return distance;
    }

    void setDistance(float new_distance) {
        HC_SR04::distance = new_distance;
    }

    static float getProbability(float distance);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_HC_SR04_H