#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive() {
    this->max_speed_straight = 0;
    this->max_speed_turn = 0;

    this->current_speed_left = 0;
    this->current_speed_right = 0;
}

DifferentialDrive::DifferentialDrive(float max_speed_straight, float max_speed_turn) {
    this->max_speed_straight = max_speed_straight;
    this->max_speed_turn = max_speed_turn / 2; //m/s (per wheel)
    this->turn_acceleration = 1000; // rad/s^2 , in the simulator it is really fast
    this->turn_deceleration = 1000; // rad/s^2, in the simulator it is really fast
    this->acceleration = 3.75;
    this->deceleration = 3.75;

    this->current_speed_left = 0;
    this->current_speed_right = 0;
}

void DifferentialDrive::setActuator(argos::CCI_PiPuckDifferentialDriveActuator *new_differentialDriveActuator) {
    this->differentialDriveActuator = new_differentialDriveActuator;
}

void DifferentialDrive::setSpeed(float speed_right, float speed_left) {
    this->current_speed_right = speed_right;
    this->current_speed_left = speed_left;
    this->differentialDriveActuator->SetLinearVelocity(speed_left, speed_right);
}

void DifferentialDrive::forward() {
    this->setSpeed(this->max_speed_straight, this->max_speed_straight);
}

void DifferentialDrive::stop() {
    this->setSpeed(0, 0);
}

void DifferentialDrive::turnLeft() {
    this->setSpeed(this->max_speed_turn, -this->max_speed_turn);
}

void DifferentialDrive::turnRight() {
    this->setSpeed(-this->max_speed_turn, this->max_speed_turn);
}
