#ifndef IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H

#include <tuple>

class Agent;

class MicroControllerBatteryManager {

public:
    MicroControllerBatteryManager() = default;

    int bytesPerNode = 36; //Number of bytes in a message (average). It depends on the sign of the x and y coordinate, and the LConfidence.
    int targetSenderIDBytes = 19; //Number of bytes prepended for the target and sender ID

    //Based on the esp32
    // From: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
    //Will change with different output powers (dBm), but we will use normal power for now
    float bluetoothTransmitConsumption_mA = 130.0; //Consumption of the microcontroller when transmitting via bluetooth
    float bluetoothReceiveConsumption_mA = 100.0; //Consumption of the microcontroller when receiving via bluetooth

    float modemSleepConsumption240MHz_ma = 68.0; // 30 mA ~ 68 mA , take worst case. Consumption of the microcontroller when the esp is in modem sleep mode (no RF) at 240MHz

    float bluetoothTransmitBandwidth_MHz = 0.9; //Bandwidth of the bluetooth transmission

    float estimateCommunicationConsumption(Agent* agent, float seconds) const;
    std::pair<float, float>  estimateTransmitConsumption(Agent* agent, float seconds) const;
    std::pair<float, float>  estimateReceiveConsumption(Agent* agent, float seconds) const;

};


#endif //IMPLEMENTATION_AND_EXAMPLES_MICROCONTROLLERBATTERYMANAGER_H
