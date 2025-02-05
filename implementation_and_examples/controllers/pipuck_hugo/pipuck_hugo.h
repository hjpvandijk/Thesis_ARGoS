/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the pi-puck.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef PIPUCK_HUGO_H
#define PIPUCK_HUGO_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering and distance actuator */
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
/* Definition of the pi-puck distance sensor */
//#include <argos3/plugins/robots/pi-puck/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include "agent_implementation/agent.h"
#include "agent_implementation/NN.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class PiPuckHugo : public CCI_Controller {

public:

    /* Class constructor. */
    PiPuckHugo();

    /* Class destructor. */
    virtual ~PiPuckHugo() {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_diffusion_controller> section.
     */
    virtual void Init(TConfigurationNode& t_node);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset() {}

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy() {}

    void BroadcastMessage(std::string message);
//    static const int nAgents = 2;
//    agent agents[nAgents];
    std::shared_ptr<Agent> agentObject;
    std::unique_ptr<DQNAgent> dqnAgent;

    double map_width = 10.0;
    double map_height = 10.0;

    Coordinate getActualAgentPosition();
    CRadians getActualAgentOrientation();



    private:

    /* Pointer to the differential steering actuator */
    CCI_PiPuckDifferentialDriveActuator* m_pcWheels;
    CCI_SimpleRadiosActuator *m_pcRadiosActuator;
//   CCI_RangeAndBearingActuator* m_pcRangeAndBearingActuator;
    /* Pointer to the pi-puck proximity sensor */
//   CCI_FootBotProximitySensor* m_pcProximity;
    CCI_SimpleRadiosSensor *m_pcRadiosSensor;
    CCI_PiPuckRangefindersSensor* m_pcRangeFindersSensor;
    CCI_PiPuckDifferentialDriveSensor* m_pcDiffDriveSensor;
    CCI_PositioningSensor* m_pcPositioningSensor;
//    CCI_RangeAndBearingSensor* m_pcRangeAndBearingSensor;


#ifdef BATTERY_MANAGEMENT_ENABLED
    Coordinate previousAgentPosition;
    CRadians previousAgentOrientation;
    int batteryMeasureTicks = 0;
    argos::CVector2 previousMovement = {0, 0};
#endif


    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><footbot_diffusion_controller> section.
     */

    double map_width_with_noise_room = 11.0;
    double map_height_with_noise_room = 11.0;

    lua_State *L;


    double directions_heatmap[512][512];
    double error_mean_heatmap[512][512];
    double orientation_offset_heatmap[512][512];


    void readHeatmapFromFile(const std::string& filename, double (&heatmap)[512][512]);

    bool mission_start = false;


    void dqnControlStep(Agent* agent);
    void execute_action(int action);
    float calculate_reward(Agent* agent);
    void train_agent();
    vec_t getLocalMapForNN(const Agent *agent);
    vec_t construct_state(Agent * agent);

        // Replay buffer
    ReplayBuffer replay_buffer;

    // Training parameters
    size_t ticks = 0;
    size_t train_interval = 100; // Train every 100 steps
    size_t batch_size = 32;

    //Previous state reward
    float previous_certainty_score = 0;
    float previous_battery_level = 100;

};

#endif