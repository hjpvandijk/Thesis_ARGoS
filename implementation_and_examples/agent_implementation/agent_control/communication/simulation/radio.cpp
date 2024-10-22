//
// Created by hugo on 18-6-24.
//

#include <argos3/core/utility/logging/argos_log.h>
#include "radio.h"

Radio::Radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor) {
    this->radioActuator = radioActuator;
    this->radioSensor = radioSensor;
}

void Radio::broadcast_message(std::string &messagePrependedWithId) const {
    messagePrependedWithId.insert(0, "<A>"); //Prepend with ALL
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    radioActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);
}

void Radio::send_message(std::string &messagePrependedWithId, const std::string& id) const {
    messagePrependedWithId.insert(0, "<" + id + ">"); //Prepend with target id
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    radioActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);
}

void Radio::receive_messages(std::vector<std::string> &messages) const {
    messages.clear();
    std::vector<argos::CByteArray> sensorMessages = radioSensor->GetInterfaces()[0].Messages;
    for (const auto& sensorMessage : sensorMessages) {
        std::string messageStr(sensorMessage.ToCArray(), sensorMessage.ToCArray() + sensorMessage.Size());
        messages.push_back(messageStr);
    }
}