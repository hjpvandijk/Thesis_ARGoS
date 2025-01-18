//
// Created by hugo on 18-6-24.
//

#ifndef ARGOS3_EXAMPLES_RADIO_H
#define ARGOS3_EXAMPLES_RADIO_H

#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
#include <queue>


class Radio {
public:
    argos::CCI_SimpleRadiosActuator *radioActuator;
    argos::CCI_SimpleRadiosSensor *radioSensor;

    Radio() = default;

    Radio(argos::CCI_SimpleRadiosActuator *radioActuator, argos::CCI_SimpleRadiosSensor *radioSensor);

    void config(float wifiTransferSpeed_Mbps, float maxJitter_ms, float message_loss_probability);

    void broadcast_message(std::string &messagePrependedWithId) const;
    void send_message(std::string &messagePrependedWithId, const std::string& id);

    void receive_messages(std::vector<std::string> &messages, double current_time_s);


private:
    float wifiTransferSpeed_Mbps; //Speed of the wifi transfer in Mbps
    float maxJitter_ms; //Max jitter in ms
    float message_loss_probability; //Probability of a message being lost

    struct MessageInTransit {
        argos::CByteArray cMessage;
        double arrive_time_s;
    };

    struct Compare {
        bool operator()(const MessageInTransit& a, const MessageInTransit& b) {
            return a.arrive_time_s > b.arrive_time_s; // Min-heap based on send_time
        }
    };

//    std::priority_queue<std::tuple<std::string, double, double>> messagesInTransit; //Message, transmission time, sent time
    std::priority_queue<MessageInTransit, std::vector<MessageInTransit>, Compare> messagesInTransit; //Message, transmission time, sent time

    void checkMessagesInTransit(std::vector<std::string> &messages, double current_time_s);

};


#endif //ARGOS3_EXAMPLES_RADIO_H
