#include "MicroControllerBatteryManager.h"

float MicroControllerBatteryManager::estimateCommunicationConsumption(Agent* agent, float seconds) {
    float transmitPower = estimateTransmitConsumption(agent, seconds);
    float receivePower = estimateReceiveConsumption(agent, seconds);
    return transmitPower + receivePower;
}

float MicroControllerBatteryManager::estimateTransmitConsumption(Agent* agent, float seconds){

    //Predict communication:
    //  - How many agents do we think there are
    //  - Have we already sent a message recently to them?



    //Translate into power usage
    //Calculate the size of the message to send
    //We know the amount of bits per node used
    //We know how many nodes per message
    //We know how many nodes in quadtree
    //So we can calculate how many messages
    //So we can use overhead to calculate the total size in bits of the messages
    //Then we can calculate how long transmitting takes
    //Then we can calculate how much power we used

    //Check if we have sent a message to this agent recently

    int nExchangeIntervalsInPeriod = std::floor( seconds/agent->QUADTREE_EXCHANGE_INTERVAL_S); //How many full exchange periods fit into the period
    double remaining = seconds - nExchangeIntervalsInPeriod * agent->QUADTREE_EXCHANGE_INTERVAL_S;

    //If the agent has sent a message to this agent recently, we will probably send a message soon
    //Else we will probably not send a message soon

    int amountOfTransmits = 0;

    for (auto &agentQuadtree : agent->agentQuadtreeSent) {
        if(agentQuadtree.second - agent->elapsed_ticks <= agent->QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second) { //If we have sent a message recently
            amountOfTransmits += nExchangeIntervalsInPeriod; //We will exchange nExchangeIntervalsInPeriod times with this agent
            if (agentQuadtree.second - agent->elapsed_ticks +
                agent->QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second <= remaining) { //If we will exchange soon
                amountOfTransmits++; //We are actually exchanging once more.
            }
        }


    }

    //Amount of nodes to exchange
    int nNodes = agent->quadtree->numberOfNodes; //It is unknown how many nodes will be added in next 'seconds' so we will use the current amount of nodes
    //Calculage amount of messages we will send
    int nNodesPerMessage = agent->quadtree->numberOfNodesPerMessage;
    int nMessages = std::floor(nNodes / nNodesPerMessage);
    int remainingNodes = nNodes - nMessages * nNodesPerMessage;
    //Calculate size of messages
    int messageSize = nNodesPerMessage * this->bytesPerMessage + this->targetSenderIDBytes;
    int remainingMessageSize = remainingNodes * this->bytesPerMessage + this->targetSenderIDBytes;

    int totalNumberOfSentBytes = nMessages * messageSize + remainingMessageSize;
    //Calculate time to send messages using the bandwidth
    float timeToTransmitS = totalNumberOfSentBytes * 8 / this->bluetoothTransmitBandwidth_MHz;

    //Calculate power usage
    float quadtreeExchangePowerUsage_mAh = timeToTransmitS * this->bluetoothTransmitConsumption_mA;

    float totalPowerUsage_mAh = quadtreeExchangePowerUsage_mAh * amountOfTransmits;

    return totalPowerUsage_mAh;
}

float MicroControllerBatteryManager::estimateReceiveConsumption(Agent* agent, float seconds){

    float totalReceivePowerUsage_mAh = 0;

    for (auto &agentQuadtree : agent->agentLocations) { // We assume agent location is received in (roughly) the same tick as the quadtree messages
        int nExchangeIntervalsInPeriod = std::floor( seconds/agent->QUADTREE_EXCHANGE_INTERVAL_S);
        double remaining = seconds - nExchangeIntervalsInPeriod * agent->QUADTREE_EXCHANGE_INTERVAL_S;

        int amountOfReceives = 0;
        if(agentQuadtree.second.second - agent->elapsed_ticks <= agent->QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second) { //If we have received a message from this agent recently
            amountOfReceives += nExchangeIntervalsInPeriod; //We will exchange nExchangeIntervalsInPeriod times with this agent
            if (agentQuadtree.second.second - agent->elapsed_ticks +
                agent->QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second <= remaining) { //If we will (probably) receive soon
                amountOfReceives++; //We are actually exchanging once more.
            }
        }

        int previousNBytesReceived = agent->agentQuadtreeBytesReceived[agentQuadtree.first]; //Amount of bytes we received from this agent previously (we use this for the calculation)
        float timeToReceiveS = previousNBytesReceived * 8 / this->bluetoothTransmitBandwidth_MHz; //Time to receive the message
        float quadtreeExchangePowerUsage_mAh = timeToReceiveS * this->bluetoothReceiveConsumption_mA; //Power usage to receive the message
        totalReceivePowerUsage_mAh += quadtreeExchangePowerUsage_mAh * amountOfReceives; //Power usage to receive all the quadtree messages from this agent added to the total
    }

    return totalReceivePowerUsage_mAh;

}

