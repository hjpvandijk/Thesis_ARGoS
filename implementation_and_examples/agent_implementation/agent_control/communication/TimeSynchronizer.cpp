#include "TimeSynchronizer.h"
#include "agent.h"


/**
 * Initiate the time synchronization by broadcasting our current time
 * @param initiating_agent
 */
void TimeSynchronizer::initTimeSync(Agent* initiating_agent){
    //determine t_txi
    int t_TXi = initiating_agent->elapsed_ticks;
    //Broadcast t_txi
    initiating_agent->broadcastMessage(t_TXiMessage(t_TXi));
}


/**
 * Received t_txi, determine t_rxj and t_txj, and send t_txi, t_rxj, t_txj
 * t_txi is the send time of the other agent
 */
void TimeSynchronizer::respondToTimeSync(std::string sender_id, Agent *receiving_agent, int t_TXi) {
    if (agentSyncs.find(sender_id) == agentSyncs.end()) { //If we have no active sync with this agent
        //Insert the new sync, and set the t_TXi and t_RXj
        int t_RXj = receiving_agent->elapsed_ticks;
        int t_TXj = t_RXj;
        agentSyncs.insert({sender_id, {t_TXj, 0, t_TXi, t_RXj}}); //Here, __j is ours, __i is the other agent's
        //Send back t_txi, t_rxj, t_txj
        //Also sending back t_txi so they know to which of their messages we responded.
        receiving_agent->sendMessage(t_TXi_t_RXj_t_TXjMessage(t_TXi, t_RXj, t_TXj), sender_id);

    }
}

/**
 * Receive t_txi, t_rxj, t_txj, determine and send t_rxi
 * Then sync the mission time
 */
void TimeSynchronizer::determineT_RXi(std::string sender_id, Agent* receiving_agent, int t_TXi, int t_RXj, int t_TXj) {
    if (agentSyncs.find(sender_id) == agentSyncs.end()) { //If we have an active sync with this agent
        //Determine t_RXi
        uint32_t t_RXi = receiving_agent->elapsed_ticks;

        agentSyncs.insert({sender_id, {t_TXi, t_RXj, t_TXj, t_RXi}}); //Here __i is ours, __j is the other agent's

        //Send t_rxi
        receiving_agent->sendMessage(t_RXiMessage(t_RXi), sender_id);

        syncMissionTime(sender_id, receiving_agent); //We have all the information to sync the mission time
    } else {
        //Highest ID wins the init, so if our ID is higher, we ignore this sync response
        if (receiving_agent->id > sender_id) {
            return;
        } else { //Else we erase the old sync (ours) and continue with this sync
            agentSyncs.erase(sender_id);
            uint32_t t_RXi = receiving_agent->elapsed_ticks;
            agentSyncs.insert({sender_id, {t_TXi, t_RXj, t_TXj, t_RXi}}); //Here __i is ours, __j is the other agent's

            //Send t_rxi
            receiving_agent->sendMessage(t_RXiMessage(t_RXi), sender_id);

            syncMissionTime(sender_id, receiving_agent); //We have all the information to sync the mission time
        }

    }
}

/**
 * Receive t_rxi
 * Then sync the mission time
 */
void TimeSynchronizer::receiveT_RXi(const std::string& sender_id, Agent* receiving_agent, int t_RXi){
    if (agentSyncs.find(sender_id) != agentSyncs.end()) { //If we have an active sync with this agent
        //Update the sync
        std::get<1>(agentSyncs[sender_id]) = t_RXi; //Here __j is ours, __i is the other agent's
    }

    syncMissionTime(sender_id, receiving_agent); //We have all the information to sync the mission time
}

/**
 * Sync the mission time with the other agent by calculating the time offset and compensating for it
 * @param other_agent_id
 * @param agent
 */
void TimeSynchronizer::syncMissionTime(const std::string& other_agent_id, Agent* agent){
    auto [t_TXi, t_RXj, t_TXj, t_RXi] = agentSyncs[other_agent_id];
    double time_offset = ((t_RXj - t_TXi) - (t_RXi - t_TXj))/2; //The difference in ticks between the agents
    int agent_compensation = std::floor(time_offset/2.0);//The amount this agent should compensate. Floor makes sure we get an integer difference, i.e. 0.5 means only one agent shifts one tick
    //Negative means agent is ahead of other agent
    //Positive means agent is behind other agent


    uint32_t synced_ticks = agent->elapsed_ticks + agent_compensation;
    agent->elapsed_ticks = synced_ticks;

    agentSyncs.erase(other_agent_id); //Remove the active sync, because we have synced the time
    lastSyncs[other_agent_id] = agent->elapsed_ticks; //Store the time we have synced with this agent
}

std::string TimeSynchronizer::t_TXiMessage(double t_TXi) {
    return "T:0|" + std::to_string(t_TXi);
}

std::string TimeSynchronizer::t_TXi_t_RXj_t_TXjMessage(double t_TXi, double t_RXj, double t_TXj) {
    return "T:1|" + std::to_string(t_TXi) + "|" + std::to_string(t_RXj) + "|" + std::to_string(t_TXj);
}

std::string TimeSynchronizer::t_RXiMessage(double t_RXi) {
    return "T:2|" + std::to_string(t_RXi);
}


double TimeSynchronizer::getLastSync(const std::string &other_agent_id){
    return lastSyncs[other_agent_id];
}


