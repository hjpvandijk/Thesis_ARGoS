/* Include the controller definition */
#include "pipuck_hugo.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "agent_implementation/agent_control/communication/simulation/Radio.h"

/****************************************/
/****************************************/

PiPuckHugo::PiPuckHugo() :
        m_pcWheels(nullptr),
        m_pcRangeFindersSensor(nullptr),
        m_pcRadiosActuator(nullptr),
        m_pcRadiosSensor(nullptr),
        agentObject(nullptr),
//   m_pcRangeAndBearingActuator(NULL),
//   m_pcRangeAndBearingSensor(NULL),
        m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_fWheelVelocity(2.5f),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                ToRadians(m_cAlpha)) {}

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
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    agentObject = std::make_shared<Agent>(Agent(this->m_strId));
    agentObject->setWifi(Radio(m_pcRadiosActuator, m_pcRadiosSensor));

    agentObject->differential_drive.setActuator(m_pcWheels);


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
    for(int i = 0; i < num_sensors; i++){
        auto sensorReading = proxReadings[i];
        double sensorNoiseM = 0.0;
        double sensorNoiseRange = agentObject->config.DISTANCE_SENSOR_NOISE_CM * 100;
        // Add noise to the sensor reading, if it is not the maximum range (nothing hit)
        if(sensorReading != agentObject->config.DISTANCE_SENSOR_PROXIMITY_RANGE) sensorNoiseM = (sensorNoiseRange - (rand() % 2 * sensorNoiseRange)) * 0.0001 ; // Random number between -1 and 1 cm (= 0.01 m), to simulate sensor noise
        agentObject->setLastRangeReadings(i, sensorReading + sensorNoiseM);
    }


//    RLOG << "Proximity readings: " << proxReadings[0] << std::endl;

//    m_pcWheels->SetLinearVelocity(0.1f, 0.1f);

    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    const auto position = positionSensorReading.Position;

    // Add noise to the sensor reading
    double positionNoiseRange = agentObject->config.POSITION_NOISE_CM / 100.0;
    double positionNoiseX = (positionNoiseRange-(rand()%2*positionNoiseRange)); // Random number between -positionNoiseRange and positionNoiseRange m, to simulate sensor noise
    double positionNoiseY = (positionNoiseRange-(rand()%2*positionNoiseRange)); // Random number between -positionNoiseRange and positionNoiseRange m, to simulate sensor noise

    agentObject->setPosition(-position.GetY() + positionNoiseX, position.GetX() + positionNoiseY); // X and Y are swapped in the positioning sensor, and we want left to be negative and right to be positive


    const auto orientation = positionSensorReading.Orientation;
    argos::CRadians orientationNoiseRange = ToRadians(argos::CDegrees(agentObject->config.ORIENTATION_NOISE_DEGREES));
    argos::CRadians orientationNoise = (orientationNoiseRange-(rand()%2*orientationNoiseRange)); ; // Random number between -orientationNoiseRange and orientationNoiseRange rad, to simulate sensor noise
    CRadians zAngle, yAngle, xAngle;
    orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    agentObject->setHeading(zAngle + orientationNoise);

    //Update agent battery level
    if (batteryMeasureTicks % 15 == 0) {

        //Get relative vector
        float traveledPathLength = sqrt(pow(agentObject->position.x - previousAgentPosition.GetX(), 2) +
                                        pow(agentObject->position.y - previousAgentPosition.GetY(), 2));
        CRadians traveledAngle = zAngle - previousAgentOrientation;
        CVector2 traveledVector = CVector2(traveledPathLength, 0).Rotate(traveledAngle);
        auto [usedPower, duration] = agentObject->batteryManager.estimateTotalPowerUsage(agentObject.get(),
                                                                                         {traveledVector});
        agentObject->batteryManager.battery.charge -= usedPower;
        argos::LOG << "Used power: " << usedPower << "mAh" << std::endl;

        previousAgentPosition = {-position.GetY(), position.GetX()};
        previousAgentOrientation = zAngle;
        batteryMeasureTicks = 0;
    }
    batteryMeasureTicks++;

//    RLOG << "Position: " << agentObject->position.x << std::endl;
//    RLOG << "Orientation: " << agentObject->heading << std::endl;
    agentObject->doStep();


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