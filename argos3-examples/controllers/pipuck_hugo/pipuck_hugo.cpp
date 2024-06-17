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
//   m_pcRangeAndBearingActuator(NULL),
//   m_pcRangeAndBearingSensor(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

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
//   m_pcRangeAndBearingActuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
////   m_pcDistanceScannerActuator = GetActuator<CCI_PiPuckDistanceScannerActuator>("pipuck_distance_scanner");
//   m_pcRangeAndBearingSensor = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
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
}

void PiPuckHugo::BroadcastMessage(std::string message) {
    UInt8* buff = (UInt8 *) message.c_str();
//        for (char c : message) {
//            cMessage << static_cast<UInt8>(c);
//            RLOG << "cmessage size: " << cMessage.Size() << std::endl;
//        }
    CByteArray cMessage = CByteArray(buff,message.size()+1);
    m_pcRadiosActuator->GetInterfaces()[0].Messages.emplace_back(cMessage);
}

/****************************************/
/****************************************/
int i = 0;

void PiPuckHugo::ControlStep() {
//    std::vector<CCI_SimpleRadiosActuator::SInterface> interfaces = m_pcRadiosActuator->GetInterfaces();
//    RLOG << "Interface id: " << interfaces[0].Id << std::endl;
////    lua_State *L;
////    m_pcRadiosActuator->CreateLuaState(L);
////
////    CCI_SimpleRadiosActuator::LuaSendMessage(L);
//
//    CByteArray cMessage;
//    cMessage.AddBuffer((UInt8*)"Hello" + toascii(i++), 5);
//    if (i==0) {

//    }
//    i++;
    BroadcastMessage("Hello");
//    std::vector<CByteArray> messages = interfaces[0].Messages;
//    messages.insert(messages.end(), message);
//    RLOG << "Message size: " << messages.size() << std::endl;
////    CByteArray byteArray = new CByteArray()
////    messages.insert(messages.begin(), CByteArray(charStr, charStr + message.size()));
//
    std::vector<CCI_SimpleRadiosSensor::SInterface> sensorInterfaces = m_pcRadiosSensor->GetInterfaces();
    std::vector<CByteArray> sensorMessages = sensorInterfaces[0].Messages;
//
    RLOG << "Sensor message size: " << sensorMessages.size() << std::endl;
    for (int i = 0; i < sensorMessages.size(); i++) {
        RLOG << "Sensor message: " << sensorMessages[i].ToCArray() << std::endl;
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
