//
// Created by hugo on 18-6-24.
//

#include <argos3/core/utility/logging/argos_log.h>
#include "radio.h"

Radio::Radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor) {
    this->radioActuator = radioActuator;
    this->radioSensor = radioSensor;
    std::make_heap(this->messagesInTransit.begin(), this->messagesInTransit.end(), Compare());
}

void Radio::config(float wifiTransferSpeed_Mbps, float maxJitter_ms, float message_loss_probability){
    this->wifiTransferSpeed_Mbps = wifiTransferSpeed_Mbps;
    this->maxJitter_ms = maxJitter_ms;
    this->message_loss_probability = message_loss_probability;
}

void Radio::broadcast_message(std::string &messagePrependedWithId) const {
    messagePrependedWithId.insert(0, "<A>"); //Prepend with ALL
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    radioActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);
}

double calculateTransmissionTime(int messageSize, double transmissionRate) {
    int udp_header_size = 8 * 8; //8 bytes
    int ip_header_size = 20 * 8; //20 bytes
    int mac_header_size = 30 * 8; //30 bytes
    messageSize += udp_header_size + ip_header_size + mac_header_size;
    return messageSize / transmissionRate;
}

double calculateJitter(int max_jitter_ms) {
    return (rand() % max_jitter_ms) * 1e-3;
}

void Radio::send_message(std::string &messagePrependedWithId, const std::string& id) {
    messagePrependedWithId.insert(0, "<" + id + ">"); //Prepend with target id
//    argos::LOG << "Prepend target id size: " << messagePrependedWithId.size() << std::endl;
    auto *buff = (argos::UInt8 *) messagePrependedWithId.c_str();
    argos::CByteArray cMessage = argos::CByteArray(buff, messagePrependedWithId.size() + 1);
    radioActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);

}

void Radio::checkMessagesInTransit(std::vector<std::string> &messages, double current_time_s) {
    while (!messagesInTransit.empty() && this->messagesInTransit.front().arrive_time_s <= current_time_s) {
        std::pop_heap(this->messagesInTransit.begin(), this->messagesInTransit.end(), Compare());
        auto messageInTransit = this->messagesInTransit.back();
        this->messagesInTransit.pop_back();
//        auto messageInTransit = this->messagesInTransit.top();
        std::string messageStr(messageInTransit.cMessage.ToCArray(),
                               messageInTransit.cMessage.ToCArray() + messageInTransit.cMessage.Size());
        messages.push_back(messageStr);
    }
//    }
}

void Radio::checkMessagesInTransitPeek(std::vector<std::string> &messages, double current_time_s) {
//    std::priority_queue<MessageInTransit, std::vector<MessageInTransit>, Compare> tempQueue = this->messagesInTransit;
    for (auto &messageInTransit: this->messagesInTransit) {
        if (messageInTransit.arrive_time_s <= current_time_s) {
            std::string messageStr(messageInTransit.cMessage.ToCArray(),
                                   messageInTransit.cMessage.ToCArray() + messageInTransit.cMessage.Size());
            messages.push_back(messageStr);
        }
    }
//    if (tempQueue.top().arrive_time_s <= current_time_s) {
//        auto messageInTransit = tempQueue.top();
//        tempQueue.pop();
//        std::string messageStr(messageInTransit.cMessage.ToCArray(),
//                               messageInTransit.cMessage.ToCArray() + messageInTransit.cMessage.Size());
//        messages.push_back(messageStr);
//    }
}


void Radio::receive_messages(std::vector<std::string> &messages, double current_time_s) {
    //First check if our messages in transit have arrived
    messages.clear();
    std::vector<argos::CByteArray> sensorMessages = radioSensor->GetInterfaces()[0].Messages;
    for (const auto &sensorMessage: sensorMessages) {
        //Random probability of dropping the message
        if ((rand() % 100) < this->message_loss_probability * 100) continue; //Ignore (drop) the message
        double transmissionTime = calculateTransmissionTime(sensorMessage.Size() * 8,
                                                            this->wifiTransferSpeed_Mbps * 1e6);
        double jitter = calculateJitter(this->maxJitter_ms);
        auto messageReceiveTime = current_time_s + transmissionTime + jitter;
        MessageInTransit message = {sensorMessage, messageReceiveTime};
        this->messagesInTransit.push_back(message);
        std::push_heap(this->messagesInTransit.begin(), this->messagesInTransit.end(), Compare());

    }

    //Check if messages have arrived
    checkMessagesInTransit(messages, current_time_s);
}

