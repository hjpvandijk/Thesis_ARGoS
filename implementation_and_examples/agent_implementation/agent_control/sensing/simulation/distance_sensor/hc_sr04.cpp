//
// Created by hugo on 16-10-24.
//

#include <algorithm>
#include <valarray>
#include "hc_sr04.h"


float HC_SR04::getError(float distance) {
    //-0,0272*x + -5,43E-03

    float a = -0.0272;
    float b = -5.43e-3;
    float error = a + b * distance;

    return error;
}

/**
 * According to data from
 * Gandha, G. I., & Nurcipto, D. (2019).
 * The Performance Improvement of the Low-Cost Ultrasonic Range Finder (HC-SR04) Using Newton's Polynomial Interpolation Algorithm.
 * Jurnal Infotel, 11(4), 108-113.
 *
 * @param actual_distance
 * @return
 */
float HC_SR04::getSimulatedMeasurement(float actual_distance) {
    //0.973x-5.43*10^(-3)
    float a = 0.973;
    float b = -5.43e-3;
    float simulated_measurement = a * actual_distance + b;
    return simulated_measurement;
}

//float HC_SR04::getError(float distance) {
//    //-1,45E-03 + 3,18E-03x + 2,11E-03x^2
//
//    float a = -1.45e-3;
//    float b = 3.18e-3;
//    float c = 2.11e-3;
//    float error = a + b * distance + c * distance * distance;
//
//
//    return error;
//}

