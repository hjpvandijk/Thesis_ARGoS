#include "BatteryManager.h"
#include "agent.h"


BatteryManager::BatteryManager(float robot_weight_kg, float robot_wheel_radius_m, float stall_torque_Nm, float no_load_rpm, float stall_current_A, float no_load_current_A, float battery_voltage, float battery_capacity) {
    this->motionSystemBatteryManager = MotionSystemBatteryManager(robot_weight_kg, robot_wheel_radius_m, stall_torque_Nm, no_load_rpm, stall_current_A, no_load_current_A);
//    this->microControllerBatteryManager = MicroControllerBatteryManager();

    battery = Battery{battery_voltage, battery_capacity, battery_capacity};

}

//float BatteryManager::EstimateBoardPowerUsage(float seconds) {
//    //Estimate how much power will be used by the IO board in the next given seconds
//    //Predict CPU usage
//    //Predict communication:
//    //  - How many agents do we think there are
//    //  - Have we already sent a message recently to them?
//    //Translate into power usage
//    return 0;
//}


/**
 * Estimate how much power will be used by the motors while traversing the given path
 * Each section of the path is relative to the former in terms of direction, the first section is relative to the current heading of the agent
 * //Also return the time it takes to traverse the path
 * @param agent
 * @param relativePath
 * @return
 */
std::tuple<float, float> BatteryManager::EstimateTotalPowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath){

    auto motionSystemPowerUsage = motionSystemBatteryManager.estimateMotorPowerUsageAndDuration(agent, relativePath); //In Wh
//    auto microControllerPowerUsage = microControllerBatteryManager.estimateCommunicationConsumption(agent, seconds);

    return motionSystemPowerUsage; //+ microControllerPowerUsage;
}
