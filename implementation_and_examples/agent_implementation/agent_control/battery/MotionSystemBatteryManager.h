#ifndef IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H


#include <tuple>
#include <argos3/core/utility/math/vector2.h>

class Agent;

class MotionSystemBatteryManager {
public:
    MotionSystemBatteryManager(float robot_weight_kg, float robot_wheel_radius_m, float robot_inter_wheel_distance_m, float stall_torque_Nm, float no_load_rpm, float stall_current_A, float no_load_current_A);
    MotionSystemBatteryManager() = default;

    float EstimateMotorPowerUsage(Agent* agent, float turnAccelerationTime, float turnFullSpeedTime, float turnDecelerationTime, float accelerationTime, float driveTime, float decelerationTime);

    std::tuple<float, float> estimateMotorPowerUsageAndDuration(Agent* agent, std::vector<argos::CVector2> relativePath);

    float getMaxAchievableSpeed() const;

private:

    //https://www.sgbotic.com/index.php?dispatch=products.view&product_id=2674
    //https://einstronic.com/product/tt-motor-yellow-geared-dc-motor/
    //https://www.verical.com/datasheet/adafruit-brushless-dc-motors-3777-5912007.pdf?srsltid=AfmBOooVLb6hED6HYiV6CVhpSCWuEZrjt1v1sG7b8DMTJIlFQQG7h-1Z
    //Based on the TT geared motor:
    //No load current 150mA @ 3V / 200mA @ 6V
    //At 3VDC we measured 150mA @ 120 RPM no-load, and 1.1 Amps when stalled
    //At 4.5VDC we measured 155mA @ 185 RPM no-load, and 1.2 Amps when stalled
    //At 6VDC we measured 160mA @ 250 RPM no-load, and 1.5 Amps when stalled
    //Stall Torque (6V): 0.8kg.cm
    //Min. Operating Speed (3V): 90RPM
    //Min. Operating Speed (6V): 200RPM


//    float robot_weight_kg = 0.5; //In kg
//    float robot_wheel_radius_m = 0.03; //In meters
//    float rolling_force_without_acceleration_N = robot_weight_kg*9.81; //In Newtons
//    float stall_torque_kg_cm = 0.8; //In kg.cm @ 6V
//    float no_load_rpm = 250; //In RPM @ 6V
//    float stall_current_A = 1.5; //In Amps @ 6V
//    float no_load_current_A = 0.16; //In Amps @ 6V

    float robot_weight_kg; //In kg
    float robot_wheel_radius_m; //In meters
    float robot_inter_wheel_distance_m = 0.0565; //In meters
    float rolling_force_without_acceleration_N; //In Newtons
    float stall_torque_kg_cm ; //In kg.cm @ 6V
    float no_load_rpm; //In RPM @ 6V
    float stall_current_A; //In Amps @ 6V
    float no_load_current_A ; //In Amps @ 6V

    float rolling_resistance_coefficient = 0.01; //Typical value for rubber on concrete

//    float forwardSpeed = 1.0;//Get speed from motor; //In m/s
//    float acceleration = 1.0;//Get acceleration from motor; //In m/s^2
//    float deceleration = 1.0;//Get deceleration from motor; //In m/s^2
//
//    float turnSpeed = 1.0;//Get turn speed from motor; //In rad/s
//    float turnAcceleration = 1.0;//Get turn acceleration from motor; //In rad/s^2
//    float turnDeceleration = 1.0;//Get turn deceleration from motor; //In rad/s^2
};


#endif //IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H
