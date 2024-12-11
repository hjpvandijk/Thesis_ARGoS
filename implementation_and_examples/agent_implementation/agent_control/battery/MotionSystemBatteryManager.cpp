#include <argos3/core/utility/logging/argos_log.h>
#include "MotionSystemBatteryManager.h"
#include "agent.h"

MotionSystemBatteryManager::MotionSystemBatteryManager(float robot_weight_kg, float robot_wheel_radius_m,
                                                       float robot_inter_wheel_distance_m,
                                                       float stall_torque_Nm,
                                                       float no_load_rpm, float stall_current_A,
                                                       float no_load_current_A) {
    this->robot_weight_kg = robot_weight_kg;
    this->robot_wheel_radius_m = robot_wheel_radius_m;
    this->robot_inter_wheel_distance_m = robot_inter_wheel_distance_m;
    this->rolling_force_without_acceleration_N = rolling_resistance_coefficient * robot_weight_kg * 9.81f;
    this->stall_torque_kg_cm = stall_torque_Nm;
    this->no_load_rpm = no_load_rpm;
    this->stall_current_A = stall_current_A;
    this->no_load_current_A = no_load_current_A;
}


float
MotionSystemBatteryManager::EstimateMotorPowerUsage(Agent *agent, float turnAccelerationTime, float turnFullSpeedTime,
                                                    float turnDecelerationTime, float accelerationTime, float driveTime,
                                                    float decelerationTime) {
    float momentOfInertia = 0.5f * robot_weight_kg * 0.0362f * 0.0362f; //In kg.m^2 , 0.0362 is the robot radius
//    float turn

    float accelerationForce = robot_weight_kg * agent->differential_drive.acceleration; //In Newtons
    float decelerationForce = robot_weight_kg * agent->differential_drive.deceleration; //In Newtons
//    float turnAccelerationForce = robot_weight_kg * agent->differential_drive.turn_acceleration; //In Newtons
//    float turnDecelerationForce = robot_weight_kg * agent->differential_drive.turn_deceleration; //In Newtons
    float turnAccelerationForce =
            momentOfInertia * agent->differential_drive.turn_acceleration / robot_wheel_radius_m; //In Newtons
    float turnDecelerationForce =
            momentOfInertia * agent->differential_drive.turn_deceleration / robot_wheel_radius_m; //In Newtons

    float totalForceDuringAcceleration = rolling_force_without_acceleration_N + accelerationForce; //In Newtons
    float totalForceDuringDeceleration = rolling_force_without_acceleration_N + decelerationForce; //In Newtons
    float totalForceDuringTurnAcceleration = rolling_force_without_acceleration_N + turnAccelerationForce; //In Newtons
    float totalForceDuringTurnDeceleration = rolling_force_without_acceleration_N + turnDecelerationForce; //In Newtons


    float forces[] = {totalForceDuringAcceleration, rolling_force_without_acceleration_N, totalForceDuringDeceleration,
                      totalForceDuringTurnAcceleration, rolling_force_without_acceleration_N,
                      totalForceDuringTurnDeceleration};
    float forceDurations[] = {accelerationTime, driveTime, decelerationTime, turnAccelerationTime, turnFullSpeedTime,
                              turnDecelerationTime};

    float totalPowerUsage = 0.0; //In Wh

    //We are assuming the motors provide the same torque (and current draw) in both directions
    for (int i = 0; i < 6; i++) {
        float force = forces[i]; //In Newtons
        float forceDuration = forceDurations[i] / 3600; //In hours
        //Add rolling resistnace
        float force_per_wheel = force / 2; //In Newtons
        float torque_per_motor = force_per_wheel * robot_wheel_radius_m; //In N.M
        float torque_per_motor_kg_cm = torque_per_motor * 10.1971621f; //In kg.cm
        //Now we know the torque, we can estimate the current draw

        //If we are not accelerating or decelerating, we can calculate the speed at said torque, and adjust the torque if the speed is higher than the max speed
        if(i == 1 || i == 4) {
            //Torque and speed are negatively linearly related: 0 torque at max speed, 0 speed at max torque (stall)
            float rpm_at_torque =
                    no_load_rpm - (torque_per_motor_kg_cm / stall_torque_kg_cm) * no_load_rpm; //In RPM @ 6V

            float speed = rpm_at_torque * 2.0f * M_PIf32 * robot_wheel_radius_m / 60.0f; //In m/s
            //Make sure the speed is not higher than the max speed, this shouldn't happen
            assert(agent->differential_drive.max_speed_straight <= speed);
            //If the achievable speed is higher than the speed we want, we need to adjust the speed and torque
            if (agent->differential_drive.max_speed_straight < speed) {
                speed = agent->differential_drive.max_speed_straight;
                //Calculate the torque at this speed
                float rpm_at_speed = speed * 60.0f / (2.0f * M_PIf32 * robot_wheel_radius_m); //In RPM @ 6V
                float torque_at_speed = (1 - rpm_at_speed / no_load_rpm) * stall_torque_kg_cm; //In kg.cm
                torque_per_motor_kg_cm = torque_at_speed;
            }
        }
        //Calculate the current using the torque
        float current_draw_A_per_motor = no_load_current_A + (stall_current_A - no_load_current_A) *
                                                             (torque_per_motor_kg_cm / stall_torque_kg_cm); //In Amps @ 6V

        //Calculate the power usage for both motors together
        float total_power = current_draw_A_per_motor * 2 * this->voltage_at_operating_speed; //In watts

        totalPowerUsage += total_power * forceDuration; //In Wh
    }


    return totalPowerUsage;
}

std::tuple<float, float> MotionSystemBatteryManager::estimateMotorPowerUsageAndDuration(Agent *agent,
                                                                                        std::vector<argos::CVector2> relativePath) {
    //Estimate how long it will take the agent to travel the path

    auto max_speed_turn_rad_s =
            agent->differential_drive.max_speed_turn / 0.0565f; //0.0565 is inter-wheel distance in meters

    float totalMotorPowerUsage = 0;
    float totalDuration = 0;
    //                                      rad/s                                           rad/s                                           rad/s/s
    float angleToAccelerateTurn =
            max_speed_turn_rad_s * max_speed_turn_rad_s / (2 * agent->differential_drive.turn_acceleration); //In rad
    float angleToDecelerateTurn =
            max_speed_turn_rad_s * max_speed_turn_rad_s / (2 * agent->differential_drive.turn_deceleration); //In rad

    float distanceToAccelerateDrive =
            agent->differential_drive.max_speed_straight * agent->differential_drive.max_speed_straight /
            (2 * agent->differential_drive.acceleration); //In meters
    float distanceToDecelerateDrive =
            agent->differential_drive.max_speed_straight * agent->differential_drive.max_speed_straight /
            (2 * agent->differential_drive.deceleration); //In meters

    for (auto &vector: relativePath) {
        float distance = vector.Length(); //In meters
        float turnAngle = std::abs(
                vector.Angle().GetValue()); //In Radians, we assume both directions yield the same power usage

        //Now we know the distance and the angle, we can estimate the time it will take to travel this path


        //We can estimate the time it will take to turn
        float totalTurnTime = 0.0;

        float timeToAccelerateTurn = 0.0;
        float timeToDecelerateTurn = 0.0;
        float timeToConstantTurn = 0.0;
        float timeToAccelerateDrive = 0.0;
        float timeToDecelerateDrive = 0.0;
        float timeToConstantDrive = 0.0;

        if (angleToAccelerateTurn + angleToDecelerateTurn > turnAngle) {
            //We can't accelerate and decelerate in time
            float maxAchievedTurnSpeed = sqrt(2 * turnAngle / (1 / agent->differential_drive.turn_acceleration + 1 /
                                                                                                                 agent->differential_drive.turn_deceleration)); //In rad/s
            timeToAccelerateTurn = maxAchievedTurnSpeed / agent->differential_drive.turn_acceleration; //In seconds
            timeToDecelerateTurn = maxAchievedTurnSpeed / agent->differential_drive.turn_deceleration; //In seconds
        } else {
            //We can accelerate and decelerate in time
            timeToAccelerateTurn = max_speed_turn_rad_s / agent->differential_drive.turn_acceleration; //In seconds
            timeToDecelerateTurn = max_speed_turn_rad_s / agent->differential_drive.turn_deceleration; //In seconds
            float angleToTravelTurn = turnAngle - angleToAccelerateTurn - angleToDecelerateTurn; //In rad
            timeToConstantTurn = angleToTravelTurn / max_speed_turn_rad_s; //In seconds
        }
        totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn + timeToConstantTurn; //In seconds

        //We can estimate the time it will take to drive
        //We can estimate the time it will take to travel the distance
        float totalDriveTime = 0.0;

        if (distanceToAccelerateDrive + distanceToDecelerateDrive > distance) {
            //We can't accelerate and decelerate in time
            float maxAchievedDriveSpeed = sqrt(2 * distance / (1 / agent->differential_drive.acceleration +
                                                               1 / agent->differential_drive.deceleration)); //In rad/s
            timeToAccelerateDrive = maxAchievedDriveSpeed / agent->differential_drive.acceleration; //In seconds
            timeToDecelerateDrive = maxAchievedDriveSpeed / agent->differential_drive.deceleration; //In seconds
        } else {
            //We can accelerate and decelerate in time
            timeToAccelerateDrive =
                    agent->differential_drive.max_speed_straight / agent->differential_drive.acceleration; //In seconds
            timeToDecelerateDrive =
                    agent->differential_drive.max_speed_straight / agent->differential_drive.deceleration; //In seconds
            float distanceToTravelDrive = distance - distanceToAccelerateDrive - distanceToDecelerateDrive; //In meters
            timeToConstantDrive = distanceToTravelDrive / agent->differential_drive.max_speed_straight; //In seconds
        }
        totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive + timeToConstantDrive; //In seconds

        totalDuration += totalDriveTime + totalTurnTime;


        //We can also estimate the power usage
        float motorPowerUsage = EstimateMotorPowerUsage(agent, timeToAccelerateTurn, timeToConstantTurn,
                                                        timeToDecelerateTurn, timeToAccelerateDrive,
                                                        timeToConstantDrive, timeToDecelerateDrive); //In Wh
        totalMotorPowerUsage += motorPowerUsage;
    }

    return {totalMotorPowerUsage, totalDuration};
}

/**
 * Get the maximum achievable speed of the robot, based on max voltage (6v)
 * @return
 */
float MotionSystemBatteryManager::getMaxAchievableSpeed() const {
    //Calculate maximum constant speed
    float force_per_wheel_constant = rolling_force_without_acceleration_N / 2; //In Newtons
    float torque_per_motor_constant = force_per_wheel_constant * robot_wheel_radius_m; //In N.M
    float torque_per_motor_kg_cm_constant = torque_per_motor_constant * 10.1971621f; //In kg.cm
    //Now we know the torque, we can estimate the current draw
    //We can estimate the current draw by looking at the torque-speed curve of the motor
    //Torque and speed are negatively linearly related: 0 torque at max speed, 0 speed at max torque (stall)
    float rpm_at_torque = no_load_rpm - (torque_per_motor_kg_cm_constant / stall_torque_kg_cm) * no_load_rpm; //In RPM @ 6V
    //TODO: decide what speed we want, and calculate the current draw at that speed

    float max_speed = rpm_at_torque * 2.0f * M_PIf32 * robot_wheel_radius_m / 60.0f; //In m/s
    //Now we know the max achievable speed.
    return max_speed;
}

/**
 * Get the voltage required for achieving the given speed in m/s
 * @param speed_m_s
 * @return
 */
float MotionSystemBatteryManager::getVoltageAtSpeed(float speed_m_s) {
    //Calculate the torque at this speed_m_s

    float rpm_at_speed = speed_m_s * 60.0f / (2.0f * M_PIf32 * robot_wheel_radius_m); //In RPM @ 6V
    float torque_at_speed = rolling_force_without_acceleration_N * robot_wheel_radius_m * 10.1971621f; //In kg.cm
    float torque_per_motor = torque_at_speed / 2; //In kg.cm

    float speed_at_torque_at_3V = 120.0f * (1-torque_per_motor/0.4); //In RPM @ 3V: no-load rpm * (1 - torque/stall torque);
    float speed_at_torque_at_4_5V = 185.0f * (1-torque_per_motor/0.6); //In RPM @ 4.5V: no-load rpm * (1 - torque/stall torque);
    float speed_at_torque_at_6V = 250.0f * (1-torque_per_motor/0.8); //In RPM @ 6V: no-load rpm * (1 - torque/stall torque);

    //Now we know the speed at the torque, we can calculate the voltage required to achieve the speed
    float voltage_at_speed_using_6v = rpm_at_speed/speed_at_torque_at_6V * 6.0f; //In Volts
    float voltage_at_speed_using_4_5v = rpm_at_speed/speed_at_torque_at_4_5V * 4.5f; //In Volts
    float voltage_at_speed_using_3v = rpm_at_speed/speed_at_torque_at_3V * 3.0f; //In Volts

    //Return the calculated voltage that is closest to the voltage used in calculation, this will be the most accurate
    float diff_6v = std::abs(voltage_at_speed_using_6v - 6.0f);
    float diff_4_5v = std::abs(voltage_at_speed_using_4_5v - 4.5f);
    float diff_3v = std::abs(voltage_at_speed_using_3v - 3.0f);

    //Interpolate values
    if(diff_6v < diff_4_5v && diff_6v < diff_3v) {
        this->voltage_at_operating_speed = voltage_at_speed_using_6v;
        //Interpolate no load current
        return voltage_at_speed_using_6v;
    } else if(diff_4_5v < diff_6v && diff_4_5v < diff_3v) {
        this->voltage_at_operating_speed = voltage_at_speed_using_4_5v;
        return voltage_at_speed_using_4_5v;
    } else {
        this->voltage_at_operating_speed = voltage_at_speed_using_3v;
        return voltage_at_speed_using_3v;
    }


}

