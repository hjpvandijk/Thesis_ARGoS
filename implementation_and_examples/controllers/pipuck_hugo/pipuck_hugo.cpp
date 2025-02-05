/* Include the controller definition */
#include "pipuck_hugo.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

PiPuckHugo::PiPuckHugo() :
        m_pcWheels(nullptr),
        m_pcRangeFindersSensor(nullptr),
        m_pcRadiosActuator(nullptr),
        m_pcRadiosSensor(nullptr),
        agentObject(nullptr)
//   m_pcRangeAndBearingActuator(NULL),
//   m_pcRangeAndBearingSensor(NULL),
   {}

/****************************************/
/****************************************/

void PiPuckHugo::Init(TConfigurationNode &t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><pipuck_diffusion><actuators> and
     * <controllers><pipuck_diffusion><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
    m_pcRadiosActuator = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
    m_pcRadiosSensor = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
    m_pcRangeFindersSensor = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
    m_pcPositioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "map_width", map_width, map_width);
    GetNodeAttributeOrDefault(t_node, "map_height", map_height, map_height);
    map_height_with_noise_room = map_height + 1.0; //Room for noise
    map_width_with_noise_room = map_width + 1.0; //Room for noise


    agentObject = std::make_shared<Agent>(Agent(this->m_strId, std::max(map_width_with_noise_room, map_height_with_noise_room)));
    agentObject->setWifi(Radio(m_pcRadiosActuator, m_pcRadiosSensor));

    agentObject->differential_drive.setActuator(m_pcWheels);

    dqnAgent = std::make_unique<DQNAgent>(agentObject.get());

    readHeatmapFromFile("controllers/pipuck_hugo/position_direction_offset.txt", this->directions_heatmap);
    readHeatmapFromFile("controllers/pipuck_hugo/orientation_offset.txt", this->orientation_offset_heatmap);
    readHeatmapFromFile("controllers/pipuck_hugo/error_heatmap.txt", this->error_mean_heatmap);

    previousAgentPosition = getActualAgentPosition();
    previousAgentOrientation = getActualAgentOrientation();
}

void PiPuckHugo::readHeatmapFromFile(const std::string& filename, double (&heatmap)[512][512]) {
    std::ifstream file(filename);
    std::string line;
    int row_index = 0;
    assert(file.good());
    while (std::getline(file, line) && row_index < 512) {
        double value;
        int col_index = 0;
        //Remove the first character, which is a '{'
        line = line.substr(1);
        //Remove the last two characters, which are a '}' and a ','
        line = line.substr(0, line.size() - 2);
        std::stringstream ss(line);
        while (ss >> value && col_index < 512) {
            heatmap[row_index][col_index] = value;
            if (ss.peek() == ',') ss.ignore();
            col_index++;
        }

        row_index++;
    }
}

int mirrored_mod(int x, int m) {
    int q = std::floor(x / m);
    if (q % 2 == 0) {
        return x % m;
    } else {
        return m - (x % m);
    }
}

/****************************************/
/****************************************/
void PiPuckHugo::ControlStep() {
    static constexpr size_t num_sensors = 4; // Define num_sensors as a static constexpr
    Real proxReadings[num_sensors] = {};
    std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> visitFn =
            [&proxReadings](const CCI_PiPuckRangefindersSensor::SInterface &sensor) {
                proxReadings[sensor.Label] = sensor.Proximity;
            };
    m_pcRangeFindersSensor->Visit(visitFn);

    int agentIdVal = 0;
    for (char c : agentObject->id) {
        agentIdVal += static_cast<int>(c);
    }
    agentIdVal *= 1234; //Multiply with some number to offset the differences between agents.

    for(int i = 0; i < num_sensors; i++){
        auto sensorReading = proxReadings[i];
        double sensorNoiseM = 0.0;
        double sensorNoiseRange = agentObject->config.DISTANCE_SENSOR_NOISE_CM * 100;
        // Add noise to the sensor reading, if it is not the maximum range (nothing hit)
        if(sensorReading != agentObject->config.DISTANCE_SENSOR_PROXIMITY_RANGE) sensorNoiseM = (sensorNoiseRange - (rand() % 2 * sensorNoiseRange)) * 0.0001 ; // Random number between -1 and 1 cm (= 0.01 m), to simulate sensor noise
        agentObject->setLastRangeReadings(i, sensorReading + sensorNoiseM);
    }


    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    const auto position = positionSensorReading.Position;

    // Simulate sensor readings with noise


    //Orientation noise
    //Agent-dependent noise
    double agentPersonalOrientationNoise = agentObject->config.ORIENTATION_JITTER_DEGREES != 0 ? mirrored_mod((agentIdVal + int((position.GetX() + position.GetY())*100)), int(agentObject->config.ORIENTATION_JITTER_DEGREES*2)) - agentObject->config.ORIENTATION_JITTER_DEGREES : 0;
//    argos::LOG << "Agent personal orientation noise: " << agentPersonalOrientationNoise << std::endl;
    argos::CRadians agentPersonalOrientationNoiseRad = ToRadians(CDegrees(agentPersonalOrientationNoise));

    // Orientation jitter
    double orientationJitterRange = agentObject->config.ORIENTATION_JITTER_DEGREES;
    argos::CRadians orientationJitter = ToRadians(CDegrees((orientationJitterRange - ((double(rand() % 200) / 100.0) * orientationJitterRange)))); ; // Random number between -orientationJitterRange and orientationJitterRange rad, to simulate sensor noise

    //Position noise
    //Agent-dependent noise
    double agentPersonalPositionNoise =  agentObject->config.POSITION_JITTER_CM != 0 ? (mirrored_mod((agentIdVal + int((position.GetX() - position.GetY())*100)), int(agentObject->config.POSITION_JITTER_CM*2)) - agentObject->config.POSITION_JITTER_CM) / 100.0 : 0;
//    argos::LOG << "Agent personal position noise: " << agentPersonalPositionNoise << std::endl;
    //Position jitter
    double positionJitterRange = agentObject->config.POSITION_JITTER_CM / 100.0;
    double positionJitter = (positionJitterRange - ((double(rand() % 200) / 100.0) * positionJitterRange)); // Random number between -positionJitterRange and positionJitterRange m, to simulate sensor noise

    //Get values from heatmap, and convert them into the correct range
    int heatmap_size = sizeof(this->directions_heatmap)/sizeof(this->directions_heatmap[0]);
    int heatmapX = std::floor((-position.GetY() + this->map_width_with_noise_room/2)/(this->map_width_with_noise_room/heatmap_size));
    int heatmapY = std::floor((position.GetX() + this->map_height_with_noise_room/2)/(this->map_height_with_noise_room/heatmap_size));

    //Simulate position estimation offset
    //Direction, including jitter and agent-dependent noise
    double direction_heatmap_value = this->directions_heatmap[heatmapY][heatmapX];
    CRadians error_direction_offset = CRadians(direction_heatmap_value) + orientationJitter + agentPersonalOrientationNoiseRad;
    //Magnitude, including jitter and agent-dependent noise
    double error_mean_heatmap_value = this->error_mean_heatmap[heatmapY][heatmapX] * agentObject->config.POSITION_NOISE_CM / 100.0;
    double error_magnitude = error_mean_heatmap_value + positionJitter + agentPersonalPositionNoise;
    CVector2 position_error_vector = CVector2(error_magnitude, 0).Rotate(error_direction_offset);
    agentObject->setPosition(-position.GetY() + position_error_vector.GetX(), position.GetX() + position_error_vector.GetY()); // X and Y are swapped in the positioning sensor, and we want left to be negative and right to be positive

    //Simulate orientation (IMU) estimation offset
    double orientation_offset_heatmap_value = this->orientation_offset_heatmap[heatmapY][heatmapX]*agentObject->config.ORIENTATION_NOISE_DEGREES;

    CRadians zAngle, yAngle, xAngle;
    const auto orientation = positionSensorReading.Orientation;
    orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    CRadians orientationOffset = ToRadians(CDegrees(orientation_offset_heatmap_value));
    agentObject->setHeading(zAngle + orientationOffset + orientationJitter + agentPersonalOrientationNoiseRad);


#ifdef BATTERY_MANAGEMENT_ENABLED
    //Update agent battery level
//    if (batteryMeasureTicks % 1 == 0) {

        //Get relative vector
        float traveledPathLength = sqrt(pow(this->getActualAgentPosition().x - previousAgentPosition.x, 2) +
                                        pow(this->getActualAgentPosition().y - previousAgentPosition.y, 2));
        CRadians traveledAngle = zAngle - previousAgentOrientation;
        CVector2 traveledVector = CVector2(traveledPathLength, 0).Rotate(traveledAngle);


        auto [usedPower, duration] = agentObject->batteryManager.calculateTotalPowerUsageFromMovement(agentObject.get(), previousMovement, traveledVector);
        agentObject->batteryManager.battery.charge -= usedPower;
//        argos::LOG << "Used power: " << usedPower << "mAh" << std::endl;

        previousAgentPosition = getActualAgentPosition();
        previousAgentOrientation = zAngle;
//        batteryMeasureTicks = 0;
//    }
//    batteryMeasureTicks++;
#endif

    if (!this->mission_start){
        agentObject->startMission();
        this->mission_start = true;
    }
//    agentObject->doStep();
    dqnControlStep(agentObject.get());


}

vec_t PiPuckHugo::construct_state(Agent * agent) {
    // 1. Get local map (10x10 grid centered on the robot)

    vec_t local_map = getLocalMapForNN(agent);

    // 2. Get proximity sensor readings
    vec_t proximity_data;
    for (const auto& sensor : agent->distance_sensors) {
        proximity_data.push_back(sensor.getDistance());
    }

    //3. Get position and orientation
    vec_t position = {float(agent->position.x), float(agent->position.y)};
    float orientation = agent->heading.GetValue(); // In Radians

    // 4. Combine all into the state vector
    vec_t state;
    state.insert(state.end(), local_map.begin(), local_map.end()); // Local map
    state.insert(state.end(), proximity_data.begin(), proximity_data.end()); // Proximity data
    state.insert(state.end(), position.begin(), position.end()); // Position
    state.push_back(orientation); // Orientation

    return state;
}

void PiPuckHugo::dqnControlStep(Agent* agent) {
    // 2. Construct the state vector
    vec_t state = construct_state(agent);

    // 3. Get action from the MARL agent
    argos::LOG << "Getting action" << std::endl;
    argos::LOG << "state size: " << state.size() << std::endl;
    int action = dqnAgent->get_action(state);

    // 4. Execute the action, and run the simulation for one step
    //Action: 0: Forward, 1: Left, 2: Right, 3: Stop
   agent->doStepAndAction(action);

    // 5. Observe the next state and reward
    vec_t next_state = construct_state(agent);
    float reward = calculate_reward(agent);

    // 6. Store experience in the replay buffer
    replay_buffer.add({state, action, reward, next_state});

    // 7. Periodically train the agent
    if (ticks++ % train_interval == 0) {
        train_agent();
    }

}


float PiPuckHugo::calculate_reward(Agent* agent) {
    // Calculate reward based on the map and agent's performance

    //Get map, calculate certainty for each cell. Reward is absolute difference between certainty and 0.5 (ambiguous)
    //Get entire map
    auto boxesAndPheromones = agent->quadtree->getAllBoxes(agent->elapsed_ticks / agent->ticks_per_second);
    //Calculate certainty for each cell: abs(pheromone - 0.5). Combine them using a squared weighted root function.
    //We do this so exploring new cells is highly rewarded, but increasing certainty by a lot is also rewarded.
    float certainty_score = 0.0f;
    for (const auto& [box, pheromone] : boxesAndPheromones) {
        certainty_score += std::sqrt(0.1f * std::abs(pheromone - 0.5));
    }

    float certainty_score_reward = certainty_score - this->previous_certainty_score;


    //Get battery level, reward is battery level, so that the agent learns to conserve energy
    float battery_level = agent->batteryManager.battery.charge;
    float battery_score_reward = this->previous_battery_level - battery_level;

    //Reward is the certainty score minus the battery used
    float reward = certainty_score_reward - battery_score_reward * 10;

    this->previous_certainty_score = certainty_score;
    this->previous_battery_level = battery_level;

    //Get mission duration, reward is duration, so that the agent learns to complete the mission quickly
    //This is implicitly done by the agent, as it will get a higher reward when it explores new areas

    return reward;
}

void PiPuckHugo::train_agent() {
    auto batch = replay_buffer.sample(batch_size);
    std::vector<vec_t> states, next_states;
    std::vector<int> actions;
    std::vector<float> rewards;

    for (const auto& exp : batch) {
        states.push_back(exp.state);
        actions.push_back(exp.action);
        rewards.push_back(exp.reward);
        next_states.push_back(exp.next_state);
    }

    dqnAgent->train(states, actions, rewards, next_states);
}

Coordinate PiPuckHugo::getActualAgentPosition() {
    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    const auto position = positionSensorReading.Position;
    return Coordinate{-position.GetY(), position.GetX()}; // X and Y are swapped in the positioning sensor, and we want left to be negative and right to be positive
}

CRadians PiPuckHugo::getActualAgentOrientation(){
    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    CRadians zAngle, yAngle, xAngle;
    const auto orientation = positionSensorReading.Orientation;
    orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    return zAngle;
}

vec_t PiPuckHugo::getLocalMapForNN(const Agent*agent) {
    auto cell_size = agent->quadtree->getResolution();
    auto local_size = std::min(2 * agent->config.FRONTIER_SEARCH_RADIUS / agent->quadtree->getResolution(), agent->quadtree->getRootBox().size / agent->quadtree->getResolution());
    vec_t local_map(local_size*local_size, 0.5f); // Initialize with default value (unexplored)

    // Define the bounding box for the local map
    float half_size = local_size / 2.0f;
    float x_min = agent->position.x - half_size;
    float y_min = agent->position.y - half_size;
//    float x_max = agent.position.x + half_size;
//    float y_max = agent.position.y + half_size;

    // Query the quadtree for cells within the bounding box
//        auto cells = query_quadtree(agenbt->quadtree, x_min, y_min, x_max, y_max);
    auto boxesAndPheromones = agent->quadtree->queryBoxesAndPheromones(agent->position, local_size, agent->elapsed_ticks / agent->ticks_per_second);

    // Fill the local map with confidence values
    for (const auto& [box, pheromone] : boxesAndPheromones) {
        auto boxCenter = box.getCenter();
        // Calculate the grid indices for the cell
        int x_start = static_cast<int>((boxCenter.x - x_min) / cell_size);
        int y_start = static_cast<int>((boxCenter.y - y_min) / cell_size);
        int x_end = static_cast<int>((boxCenter.x + box.size - x_min) / cell_size);
        int y_end = static_cast<int>((boxCenter.y + box.size - y_min) / cell_size);

        // Fill the local map with the cell's confidence value
        for (int i = x_start; i < x_end; ++i) {
            for (int j = y_start; j < y_end; ++j) {
                if (i >= 0 && i < local_size && j >= 0 && j < local_size) {
                    local_map[i * local_size + j] = pheromone;
                }
            }
        }
    }

    return local_map;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(PiPuckHugo, "pipuck_hugo_controller")