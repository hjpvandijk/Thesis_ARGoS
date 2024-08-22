//
// Created by hugo on 18-6-24.
//

#ifndef ARGOS3_EXAMPLES_RADIO_H
#define ARGOS3_EXAMPLES_RADIO_H

#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>


class Radio {
public:
    argos::CCI_SimpleRadiosActuator *radioActuator;
    argos::CCI_SimpleRadiosSensor *radioSensor;

    Radio() {}

    Radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor);

    void broadcast_message(argos::CByteArray &message) const;

    void receive_messages(std::vector<std::string> *messages) const;

};


#endif //ARGOS3_EXAMPLES_RADIO_H
