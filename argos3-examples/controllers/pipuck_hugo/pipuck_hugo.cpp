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
        m_pcWheels(NULL),
        m_pcRangeFindersSensor(NULL),
        m_pcRadiosActuator(NULL),
        m_pcRadiosSensor(NULL),
        agentObject(NULL),
//   m_pcRangeAndBearingActuator(NULL),
//   m_pcRangeAndBearingSensor(NULL),
   m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_fWheelVelocity(2.5f),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)){}

/****************************************/
/****************************************/

void PiPuckHugo::Init(TConfigurationNode& t_node) {
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
   m_pcWheels    = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
   m_pcRadiosActuator = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
   m_pcRadiosSensor = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
   m_pcRangeFindersSensor = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
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

   agentObject = new agent(this->m_strId);
   agentObject->setWifi(radio(m_pcRadiosActuator, m_pcRadiosSensor));

    L = luaL_newstate();
    luaL_openlibs(L);

    // Create a new table for rangefinders
    lua_newtable(L);
    lua_setglobal(L, "rangefinders");

    // Create subtables for each rangefinder (1-based indexing)
    lua_getglobal(L, "rangefinders");
    for (int i = 1; i <= 3; ++i) {
        lua_newtable(L);
        lua_rawseti(L, -2, i);
    }
    lua_pop(L, 1);


}

//void PiPuckHugo::BroadcastMessage(std::string message) {
//    UInt8* buff = (UInt8 *) message.c_str();
////        for (char c : message) {
////            cMessage << static_cast<UInt8>(c);
////            RLOG << "cmessage size: " << cMessage.Size() << std::endl;
////        }
//    CByteArray cMessage = CByteArray(buff,message.size()+1);
//    m_pcRadiosActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);
//}

/****************************************/
/****************************************/
int i = 0;

void PiPuckHugo::ControlStep() {
    RLOG << agentObject->position->toString() << std::endl;
    agentObject->broadcastMessage("Hello from agent " + agentObject->getId());
    agentObject->readMessages();
    std::vector<std::string> messages = agentObject->getMessages();

    if(i==0) {
        // Call the function to populate the Lua state
        lua_getglobal(L, "rangefinders");
        m_pcRangeFindersSensor->ReadingsToLuaState(L);
        lua_pop(L, 1);
        // Retrieve proximity values
        lua_getglobal(L, "rangefinders");
        for (int i = 1; i <= 3; ++i) {
            lua_rawgeti(L, -1, i);
            lua_getfield(L, -1, "proximity");
            double proximity = lua_tonumber(L, -1);
            lua_pop(L, 2); // pop proximity and the subtable
            RLOG << "Rangefinder " << i << " proximity: " << proximity << std::endl;
        }
        lua_pop(L, 1); // pop rangefinders
        lua_close(L);
    }

//    m_pcRangeAndBearingActuator->SetData(message);

//    std::vector<CCI_RangeAndBearingSensor::SPacket> readings = m_pcRangeAndBearingSensor->GetReadings();
//    RLOG << "Readings: " << readings.size() << std::endl;

    /* Get readings from proximity sensor */
//   const CCI_P::TReadingsMap tDistReads = m_pcDistanceScanner->GetReadingsMap();
////   RLOG << "Readings: " << tDistReads.size() << std::endl;
//   /* Sum them together */
//   CVector2 cAccumulator;
////   Real maxLength = -1.0f;
////   CRadians maxAngle = CRadians::ZERO;
////   for(size_t i = 0; i < tProxReads.size(); ++i) {
////      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
////        if(tProxReads[i].Value > maxLength) {
////             maxLength = tProxReads[i].Value;
////             maxAngle = tProxReads[i].Angle;
////        }
////   }
////   RLOG << "Max length: " << maxLength << " at angle " << maxAngle << std::endl;
////   cAccumulator /= tProxReads.size();
//   /* If the angle of the vector is small enough and the closest obstacle
//    * is far enough, continue going straight, otherwise curve a little
//    */
//
//   CRadians cAngle = cAccumulator.Angle();
//   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
//      cAccumulator.Length() < m_fDelta ) {
//      /* Go straight */
//      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
//   }
////   else {
//      /* Turn, depending on the sign of the angle */
//      if(rand()%100> 50) {
//         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
//      }
//      else {
//         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
//      }
//   }
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
