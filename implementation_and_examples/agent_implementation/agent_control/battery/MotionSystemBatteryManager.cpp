#include "MotionSystemBatteryManager.h"

float MotionSystemBatteryManager::EstimateMotorPowerUsage(double distance, double turns) {
    //Something like:
    //Know motor amperage at speed and load ??
    //Know voltage at speed and load ??
    //Know speed at intervals
    //Assume constant load
    //Calculate power usage at each interval:
    //P = (V * I) / 2
    //Sum all power usages



    return 0;
}

std::tuple<float, float> MotionSystemBatteryManager::estimateMotorPowerUsageAndDuration(std::vector<argos::CVector2> relativePath) {
    //Estimate how long it will take the agent to travel the path

    float totalMotorPowerUsage = 0;
    double totalDuration = 0;

    for(auto &vector: relativePath) {
        auto distance = vector.Length(); //In meters
        auto turnAngle = vector.Angle().GetValue(); //In Radians

        //Now we know the distance and the angle, we can estimate the time it will take to travel this path
        auto forwardSpeed = 1.0;//Get speed from motor; //In m/s
        auto acceleration = 1.0;//Get acceleration from motor; //In m/s^2
        auto deceleration = 1.0;//Get deceleration from motor; //In m/s^2
        auto turnSpeed = 1.0;//Get turn speed from motor; //In rad/s
        auto turnAcceleration = 1.0;//Get turn acceleration from motor; //In rad/s^2
        auto turnDeceleration = 1.0;//Get turn deceleration from motor; //In rad/s^2

        //We can estimate the time it will take to turn
        auto totalTurnTime = 0.0;
        auto angleToAccelerateTurn = turnSpeed * turnSpeed / (2 * turnAcceleration); //In rad
        auto angleToDecelerateTurn = turnSpeed * turnSpeed / (2 * turnDeceleration); //In rad
        if(angleToAccelerateTurn + angleToDecelerateTurn > turnAngle) {
            //We can't accelerate and decelerate in time
            auto maxAchievedTurnSpeed =sqrt(2)*sqrt(turnAngle) / sqrt(turnAcceleration + turnDeceleration); //In rad/s
            auto timeToAccelerateTurn = maxAchievedTurnSpeed / turnAcceleration; //In seconds
            auto timeToDecelerateTurn = maxAchievedTurnSpeed / turnDeceleration; //In seconds
            totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn; //In seconds
        } else {
            //We can accelerate and decelerate in time
            auto timeToAccelerateTurn = turnSpeed / turnAcceleration; //In seconds
            auto timeToDecelerateTurn = turnSpeed / turnDeceleration; //In seconds
            auto angleToTravelTurn = turnAngle - angleToAccelerateTurn - angleToDecelerateTurn; //In rad
            auto timeToConstantTurn = angleToTravelTurn / turnSpeed; //In seconds
            totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn + timeToConstantTurn; //In seconds
        }
        //We can estimate the time it will take to drive
        //We can estimate the time it will take to travel the distance
        auto totalDriveTime = 0.0;
        auto distanceToAccelerateDrive = forwardSpeed*forwardSpeed / (2*acceleration); //In meters
        auto distanceToDecelerateDrive = forwardSpeed*forwardSpeed / (2*deceleration); //In meters
        if(distanceToAccelerateDrive + distanceToDecelerateDrive > distance) {
            //We can't accelerate and decelerate in time
            auto maxAchievedDriveSpeed = sqrt(2) * sqrt(distance) / sqrt(acceleration + deceleration); //In m/s
            auto timeToAccelerateDrive = maxAchievedDriveSpeed / acceleration; //In seconds
            auto timeToDecelerateDrive = maxAchievedDriveSpeed / deceleration; //In seconds
            totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive; //In seconds
        } else {
            //We can accelerate and decelerate in time
            auto timeToAccelerateDrive = forwardSpeed / acceleration; //In seconds
            auto timeToDecelerateDrive = forwardSpeed / deceleration; //In seconds
            auto distanceToTravelDrive = distance - distanceToAccelerateDrive - distanceToDecelerateDrive; //In meters
            auto timeToConstantDrive = distanceToTravelDrive / forwardSpeed; //In seconds
            totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive + timeToConstantDrive; //In seconds
        }

        totalDuration += totalDriveTime + totalTurnTime;


        //We can also estimate the power usage
        auto motorPowerUsage = EstimateMotorPowerUsage(distance, turnAngle); //In watts
        totalMotorPowerUsage += motorPowerUsage;
    }
}
