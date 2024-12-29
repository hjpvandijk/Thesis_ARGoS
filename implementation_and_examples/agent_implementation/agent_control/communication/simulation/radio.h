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

    Radio() = default;

    Radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor);

    void broadcast_message(std::string &messagePrependedWithId) const;
    void send_message(std::string &messagePrependedWithId, const std::string& id) const;

    void receive_messages(std::vector<std::string> &messages) const;

private:
    float wifiTransferSpeed_Mbps = 10; //Speed of the wifi transfer in Mbps
    float maxJitter_ms = 10; //Max jitter in ms

};


#endif //ARGOS3_EXAMPLES_RADIO_H
