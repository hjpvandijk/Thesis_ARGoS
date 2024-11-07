#include "BatteryManager.h"

float BatteryManager::EstimateBoardPowerUsage(float seconds) {
    //Estimate how much power will be used by the IO board in the next given seconds
    //Predict CPU usage
    //Predict communication:
    //  - How many agents do we think there are
    //  - Have we already sent a message recently to them?
    //Translate into power usage
    return 0;
}



float BatteryManager::EstimateTotalPowerUsage(std::vector<argos::CVector2> relativePath) {


    float totalBoardPowerUsage = EstimateBoardPowerUsage(totalDuration); //In watts

    return 0;
}
