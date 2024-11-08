#include "BatteryManager.h"
#include "agent.h"

void BatteryManager::setBatterySensor(argos::CCI_BatterySensor *newBatterySensor) {
    this->batterySensor = newBatterySensor;
    this->batterySensor->Enable();
}

double BatteryManager::getStateOfCharge() {
    auto batteryReading =  batterySensor->GetReading();
    return batteryReading.AvailableCharge;
}

/**
 * Estimate the power usage (in % from full) of the agent for following the given path
 * Only considers the power usage of the motors, not the power usage of the sensors, communication, etc.
 * Specific for the Pi-Puck robot
 * @param agent
 * @param relativePath
 * @return
 */
double BatteryManager::estimatePowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath) {
    if(getStateOfCharge() > 0.0) {

        auto timeSpentAtSpeeds =  calculateTimeAtSpeeds(agent, relativePath); // For each achieved speed (with intervals), the time the robot spends at that speed
        double powerUsage = 0.0; //In % from full
        auto fBat = getStateOfCharge();

        //For each speed, calculate the power usage
        for (auto &timeAtSpeed: timeSpentAtSpeeds) {
            auto speed = std::get<0>(timeAtSpeed); // cm/s
            auto fSpeed = std::round(std::max(0.0, std::min(0.150, speed)) * 10000.0) / 100.0; // cm/s
            auto ratio = fSpeed ==0 ?  0 : (speed*100/fSpeed);
            auto time = std::get<1>(timeAtSpeed); // s



            int spdL = std::max(0.0, floor(fSpeed));
            int spdH = std::min(15.0, ceil(fSpeed));


            // First interpolation point: LOW
            double m1, h1;
            if (fBat <= M1[spdL][4] && fBat >= M1[spdL][5]) {
                h1 = M1[spdL][0];
                m1 = M1[spdL][1];
            } else if (fBat <= M2[spdL][4] && fBat >= M2[spdL][5]) {
                h1 = M2[spdL][0];
                m1 = M2[spdL][1];
            } else if (fBat <= M3[spdL][4] && fBat >= M3[spdL][5]) {
                h1 = M3[spdL][0];
                m1 = M3[spdL][1];
            } else {
                h1 = M4[spdL][0];
                m1 = M4[spdL][1];
            }

            double x1 = (fBat - h1) / m1;
            double y1 = m1 * (x1 + time) + h1;

            if (spdL == spdH) {
                powerUsage += (fBat - std::max(0.0, y1)) * ratio;
            } else {
                // Second interpolation point: RIGHT
                double m2, h2;
                if (fBat <= M1[spdH][4] && fBat >= M1[spdH][5]) {
                    h2 = M1[spdH][0];
                    m2 = M1[spdH][1];
                } else if (fBat <= M2[spdH][4] && fBat >= M2[spdH][5]) {
                    h2 = M2[spdH][0];
                    m2 = M2[spdH][1];
                } else if (fBat <= M3[spdH][4] && fBat >= M3[spdH][5]) {
                    h2 = M3[spdH][0];
                    m2 = M3[spdH][1];
                } else {
                    h2 = M4[spdH][0];
                    m2 = M4[spdH][1];
                }

                double x2 = (fBat - h2) / m2;
                double y2 = m2 * (x2 + time) + h2;

                double d = (fSpeed - spdL) / (spdH - spdL);
                if (y1 < y2) {
                    powerUsage += (fBat - std::max(0.0, y1 + (y2 - y1) * d)) * ratio;
                } else {
                    powerUsage += (fBat - std::max(0.0, y2 + (y1 - y2) * d)) * ratio;
                }
            }
            fBat = getStateOfCharge() - powerUsage; //Battery level after performing operation
        }
        return powerUsage;
    }
    return 0.0; //Battery is empty
}

void updateAccelerationTimeSpeeds(std::vector<std::tuple<double,double>>& timeAtEachSpeed, double duration, double acceleration, double accelDecelSpeedcheckIntervalS) {
    for (auto time = 0.0; time<=duration; time+=accelDecelSpeedcheckIntervalS) {
        auto speed = time*acceleration;
        timeAtEachSpeed.emplace_back(speed,accelDecelSpeedcheckIntervalS);
    }
}

void updateDecelerationTimeSpeeds(std::vector<std::tuple<double,double>>& timeAtEachSpeed, double duration, double acceleration, double accelDecelSpeedcheckIntervalS) {
    for (auto time = 0.0; time<=duration; time+=accelDecelSpeedcheckIntervalS) {
        auto speed = duration*acceleration - time*acceleration;
        timeAtEachSpeed.emplace_back(speed,accelDecelSpeedcheckIntervalS);
    }
}

void updateConstantSpeedTime(std::vector<std::tuple<double,double>>& timeAtEachSpeed, double speed, double duration) {
    timeAtEachSpeed.emplace_back(speed,duration);
}

std::vector<std::tuple<double,double>> BatteryManager::calculateTimeAtSpeeds(Agent* agent, std::vector<argos::CVector2> relativePath) {
    //Estimate how we are at certain speeds while progressing the given path

    std::vector<std::tuple<double,double>> timeAtEachSpeed; // vector<speed,time>

    auto forwardSpeed = agent->differential_drive.max_speed_straight;//Get speed from motor; //In cm/s
    auto acceleration = 1.0;//Get acceleration from motor; //In cm/s^2
    auto deceleration = 1.0;//Get deceleration from motor; //In cm/s^2
    auto turnSpeed = agent->differential_drive.max_speed_turn;//Get turn speed from motor; //In rad/s
    auto turnAcceleration = 1.0;//Get turn acceleration from motor; //In rad/s^2
    auto turnDeceleration = 1.0;//Get turn deceleration from motor; //In rad/s^2

    auto totalDuration = 0.0;


    for(auto &vector: relativePath) {
        auto distance = vector.Length(); //In meters
        auto turnAngle = vector.Angle().GetValue(); //In Radians

        //Now we know the distance and the angle, we can estimate the time it will take to travel this path


        //We can estimate the time it will take to turn
        auto totalTurnTime = 0.0;
        auto angleToAccelerateTurn = turnSpeed * turnSpeed / (2 * turnAcceleration); //In rad
        auto angleToDecelerateTurn = turnSpeed * turnSpeed / (2 * turnDeceleration); //In rad

        auto timeToDecelerateTurn = 0.0;

        if(angleToAccelerateTurn + angleToDecelerateTurn > turnAngle) {
            //We can't accelerate and decelerate in time
            auto maxAchievedTurnSpeed =sqrt(2)*sqrt(turnAngle) / sqrt(turnAcceleration + turnDeceleration); //In rad/s
            auto timeToAccelerateTurn = maxAchievedTurnSpeed / turnAcceleration; //In seconds
            timeToDecelerateTurn = maxAchievedTurnSpeed / turnDeceleration; //In seconds
            totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn; //In seconds
            updateAccelerationTimeSpeeds(timeAtEachSpeed, timeToAccelerateTurn, turnAcceleration, this->accelDecelSpeedcheckIntervalS);

        } else {
            //We can accelerate and decelerate in time
            auto timeToAccelerateTurn = turnSpeed / turnAcceleration; //In seconds
            timeToDecelerateTurn = turnSpeed / turnDeceleration; //In seconds
            auto angleToTravelTurn = turnAngle - angleToAccelerateTurn - angleToDecelerateTurn; //In rad
            auto timeToConstantTurn = angleToTravelTurn / turnSpeed; //In seconds
            totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn + timeToConstantTurn; //In seconds
            updateAccelerationTimeSpeeds(timeAtEachSpeed, timeToAccelerateTurn, turnAcceleration, this->accelDecelSpeedcheckIntervalS);
            updateConstantSpeedTime(timeAtEachSpeed, turnSpeed, timeToConstantTurn);
        }

        updateDecelerationTimeSpeeds(timeAtEachSpeed, timeToDecelerateTurn, turnDeceleration, this->accelDecelSpeedcheckIntervalS);


        //We can estimate the time it will take to drive
        //We can estimate the time it will take to travel the distance
        auto totalDriveTime = 0.0;
        auto distanceToAccelerateDrive = forwardSpeed*forwardSpeed / (2*acceleration); //In meters
        auto distanceToDecelerateDrive = forwardSpeed*forwardSpeed / (2*deceleration); //In meters

        auto timeToDecelerateDrive = 0.0;
        if(distanceToAccelerateDrive + distanceToDecelerateDrive > distance) {
            //We can't accelerate and decelerate in time
            auto maxAchievedDriveSpeed = sqrt(2) * sqrt(distance) / sqrt(acceleration + deceleration); //In m/s
            auto timeToAccelerateDrive = maxAchievedDriveSpeed / acceleration; //In seconds
            timeToDecelerateDrive = maxAchievedDriveSpeed / deceleration; //In seconds
            totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive; //In seconds
            updateAccelerationTimeSpeeds(timeAtEachSpeed, timeToAccelerateDrive, acceleration, this->accelDecelSpeedcheckIntervalS);
        } else {
            //We can accelerate and decelerate in time
            auto timeToAccelerateDrive = forwardSpeed / acceleration; //In seconds
            timeToDecelerateDrive = forwardSpeed / deceleration; //In seconds
            auto distanceToTravelDrive = distance - distanceToAccelerateDrive - distanceToDecelerateDrive; //In meters
            auto timeToConstantDrive = distanceToTravelDrive / forwardSpeed; //In seconds
            totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive + timeToConstantDrive; //In seconds
            updateAccelerationTimeSpeeds(timeAtEachSpeed, timeToAccelerateDrive, acceleration, this->accelDecelSpeedcheckIntervalS);
            updateConstantSpeedTime(timeAtEachSpeed, forwardSpeed, timeToConstantDrive);
        }
        updateDecelerationTimeSpeeds(timeAtEachSpeed, timeToDecelerateDrive, deceleration, this->accelDecelSpeedcheckIntervalS);

        totalDuration += totalDriveTime + totalTurnTime;


    }
    return timeAtEachSpeed;
}
