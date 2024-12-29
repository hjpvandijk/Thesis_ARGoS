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

double calculateTransmissionTime(double messageSize, double transmissionRate) {
    return messageSize / transmissionRate;
}

double calculateJitter(int max_jitter_ms) {
    return (rand() % max_jitter_ms) * 1e-3;
}

void simulateTransmissionAndJitter(double transmissionTime, double jitter) {
    //    sleep(transmissionTime + jitter); TODO: Implement threads so we can sleep
}

void Radio::send_message(std::string &messagePrependedWithId, const std::string& id) const {
    messagePrependedWithId.insert(0, "<" + id + ">"); //Prepend with target id
    argos::LOG << "Prepend target id size: " << messagePrependedWithId.size() << std::endl;
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    double transmissionTime = calculateTransmissionTime(messagePrependedWithId.size(), this->wifiTransferSpeed_Mbps * 1e6);
    double jitter = calculateJitter(this->maxJitter_ms);
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    argos::LOG << "Cmessage size: " << cMessage.Size() << std::endl;
    simulateTransmissionAndJitter(transmissionTime, jitter);
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

