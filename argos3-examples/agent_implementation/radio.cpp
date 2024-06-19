//
// Created by hugo on 18-6-24.
//

#include <argos3/core/utility/logging/argos_log.h>
#include "radio.h"

radio::radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor) {
    this->radioActuator = radioActuator;
    this->radioSensor = radioSensor;
}

void radio::broadcast_message(argos::CByteArray &message) const {
    radioActuator->GetInterfaces()[0].Messages.emplace_back(message);
}

void radio::receive_messages(std::vector<std::string> *messages) const {
    messages->clear();
    std::vector<argos::CByteArray> sensorMessages = radioSensor->GetInterfaces()[0].Messages;
    for (const auto& sensorMessage : sensorMessages) {
        std::string messageStr(sensorMessage.ToCArray(), sensorMessage.ToCArray() + sensorMessage.Size());
        messages->push_back(messageStr);
    }
}