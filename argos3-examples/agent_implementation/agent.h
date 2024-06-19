//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "coordinate.h"
#include "radio.h"
#include <string>


class agent {
    public:
        std::string id{};
        coordinate *position{};
        double speed{};
        radio wifi;


        //Distance sensor
        //Infrared sensor

        //Some sort of map or grid to keep track of the environment
        //Some sort of list of agents to keep track of other agents


        agent() {}

        explicit agent(std::string id);

        agent(std::string id, coordinate *position);

        void setPosition(double new_x, double new_y) const;

        void setPosition(coordinate *position);

        coordinate *getPosition() const;

        std::string getId() const;

        void setId(std::string id);

        void setSpeed(double speed);

        double getSpeed() const;

        radio getWifi() const;

        void setWifi(radio wifi);

        void print();

        void updateMap();

        void readDistanceSensor();

        void readInfraredSensor();

        void calculateNextPosition();

        void broadcastMessage(std::string message);

        void readMessages();

        std::vector<std::string> getMessages();


private:
    std::vector<std::string> *messages;

    };


#endif //THESIS_ARGOS_AGENT_H
