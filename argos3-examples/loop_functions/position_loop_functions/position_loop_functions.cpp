#include "position_loop_functions.h"
#include "controllers/pipuck_hugo/pipuck_hugo.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>

/****************************************/
/****************************************/

CPositionLoopFunctions::CPositionLoopFunctions()
   {
}


/****************************************/
/****************************************/

void CPositionLoopFunctions::PreStep() {
    /* Logic to update the position of the pipucks */
    CSpace::TMapPerType &m_cPipucks = GetSpace().GetEntitiesByType("pipuck");


    for (CSpace::TMapPerType::iterator it = m_cPipucks.begin();
         it != m_cPipucks.end();
         ++it) {
        /* Get handle to pi-puck entity and controller */
        CPiPuckEntity &cPiPuck = *any_cast<CPiPuckEntity *>(it->second);
        PiPuckHugo &cController = dynamic_cast<PiPuckHugo &>(cPiPuck.GetControllableEntity().GetController());

        /* Get the position of the pi-puck on the ground as a CVector2 */
        CVector2 cPos;
        cPos.Set(cPiPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cPiPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        cController.agentObject->setPosition(cPos.GetX(), cPos.GetY());



    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CPositionLoopFunctions, "position_loop_functions")
