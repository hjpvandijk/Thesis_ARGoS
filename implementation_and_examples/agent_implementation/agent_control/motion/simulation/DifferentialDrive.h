//
// Created by hugo on 22-10-24.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_DIFFERENTIALDRIVE_H
#define IMPLEMENTATION_AND_EXAMPLES_DIFFERENTIALDRIVE_H

#include "argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h"

class DifferentialDrive {
public:
    float max_speed_straight;
    float max_speed_turn;


    float current_speed_right{};
    float current_speed_left{};

    DifferentialDrive();
    DifferentialDrive(float max_speed_straight, float max_speed_turn);


    void setActuator(argos::CCI_PiPuckDifferentialDriveActuator *differentialDriveActuator);
    void setSpeed(float speed_right, float speed_left);
    void forward();
    void stop();
    void turnLeft();
    void turnRight();


private:
    argos::CCI_PiPuckDifferentialDriveActuator *differentialDriveActuator{};

};


#endif //IMPLEMENTATION_AND_EXAMPLES_DIFFERENTIALDRIVE_H
