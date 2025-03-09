//
// Created by hugo on 16-10-24.
//

#include <algorithm>
#include <valarray>
#include "hc_sr04.h"

float HC_SR04::getProbability(float distance) {
    float error = getError(distance);

    float probability = 1.0 - pow(10*error, 2); //Decrease probability

    return std::max(0.0f, std::min(1.0f, probability));

}

float HC_SR04::getError(float distance) {
    //-1,45E-03 + 3,18E-03x + 2,11E-03x^2

    float a = -1.45e-3;
    float b = 3.18e-3;
    float c = 2.11e-3;
    float error = a + b * distance + c * distance * distance;


    return error;
}

