#ifndef IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H


#include <tuple>
#include <argos3/core/utility/math/vector2.h>

class MotionSystemBatteryManager {
public:
    float EstimateMotorPowerUsage(double distance, double turns);

    std::tuple<float, float> estimateMotorPowerUsageAndDuration(std::vector<argos::CVector2> relativePath);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H
