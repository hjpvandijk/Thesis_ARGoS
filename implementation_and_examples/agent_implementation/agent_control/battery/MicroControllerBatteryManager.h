#ifndef IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H


#include "agent.h"

class MicroControllerBatteryManager {

public:

    int bytesPerMessage = 36; //Number of bytes in a message (average). It depends on the sign of the x and y coordinate, and the LConfidence.
    int targetSenderIDBytes = 19; //Number of bytes prepended for the target and sender ID

    //Based on the esp32
// From: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
    const float bluetoothTransmitConsumption_mA = 130.0; //Consumption of the microcontroller when transmitting via bluetooth
    const float bluetoothReceiveConsumption_mA = 100.0; //Consumption of the microcontroller when receiving via bluetooth
    const float modemSleepConsumption240MHz_ma = 49.0; //Consumption of the microcontroller when the esp is in modem sleep mode at 240MHz

    const float bluetoothTransmitBandwidth_MHz = 0.9; //Bandwidth of the bluetooth transmission

    void estimateCPUConsumption(float seconds);
    float estimateCommunicationConsumption(Agent* agent, float seconds);
    float estimateTransmitConsumption(Agent* agent, float seconds);
    float estimateReceiveConsumption(Agent* agent, float seconds);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H
