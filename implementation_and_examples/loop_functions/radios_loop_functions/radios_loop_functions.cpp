#include "radios_loop_functions.h"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CRadiosLoopFunctions::PostStep() {
      /* wait five ticks before evaluating the test */
//      if(GetSpace().GetSimulationClock() < 5) {
//         return;
//      }
//      else {
         CPiPuckEntity* pcNode0 = nullptr;
          CPiPuckEntity* pcNode1 = nullptr;
         try {
            pcNode0 = dynamic_cast<CPiPuckEntity*>(&GetSpace().GetEntity("pipuck1"));
            pcNode1 = dynamic_cast<CPiPuckEntity*>(&GetSpace().GetEntity("pipuck2"));
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Cell was not added to the simulator", ex);
         }
         if(pcNode0 == nullptr) {
            THROW_ARGOSEXCEPTION("pipuck1 was not a prototype");
         }
         if(pcNode1 == nullptr) {
            THROW_ARGOSEXCEPTION("pipuck2 was not a prototype");
         }
         CCI_SimpleRadiosSensor* pcSimpleRadioSensorNode0 =
            pcNode0->GetControllableEntity().GetController().GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
         CCI_SimpleRadiosSensor* pcSimpleRadioSensorNode1 =
            pcNode1->GetControllableEntity().GetController().GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
         LOG << pcSimpleRadioSensorNode0->GetInterfaces()[0].Id << std::endl;
         LOG << "pipuck1: " << pcSimpleRadioSensorNode0->GetInterfaces()[0].Messages.size() << std::endl;
         LOG << "pipuck2: " << pcSimpleRadioSensorNode1->GetInterfaces()[0].Messages.size() << std::endl;
//         if(pcSimpleRadioSensorNode0->GetInterfaces()[0].Messages.size() != 0) {
//            THROW_ARGOSEXCEPTION("pipuck1 should have zero messages");
//         }
//         if(pcSimpleRadioSensorNode1->GetInterfaces()[0].Messages.size() != 1) {
//            THROW_ARGOSEXCEPTION("pipuck2 should have one message");
//         }
//         return;
//      }
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CRadiosLoopFunctions, "radios_loop_functions");

}
