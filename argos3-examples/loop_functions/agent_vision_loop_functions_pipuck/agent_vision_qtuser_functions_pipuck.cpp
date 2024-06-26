#include "agent_vision_qtuser_functions_pipuck.h"
#include "agent_vision_loop_functions_pipuck.h"

/****************************************/
/****************************************/

CAgentVisionQTUserFunctions::CAgentVisionQTUserFunctions() :
        m_cAgVisLF(dynamic_cast<CAgentVisionLoopFunctions &>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CAgentVisionQTUserFunctions::DrawInWorld() {
    /* Go through all the robot waypoints and draw them */
    for(std::map<CPiPuckEntity*, CVector3>::const_iterator it = m_cAgVisLF.GetAgentCoordinates().begin();
        it != m_cAgVisLF.GetAgentCoordinates().end();
        ++it){
//        DrawBox(it->second, CQuaternion(), 0.1f, CColor::GRAY50);
        DrawBox(it->second, CQuaternion(), CVector3(4,4,0), CColor::GRAY80);
    }
    for (CAgentVisionLoopFunctions::TCoordinateMap::const_iterator it = m_cAgVisLF.GetObjectCoordinates().begin();
         it != m_cAgVisLF.GetObjectCoordinates().end();
         ++it) {
        DrawCoordinates(it->second, CColor::RED);
    }
    for(CAgentVisionLoopFunctions::TCoordinateMap ::const_iterator it = m_cAgVisLF.GetOtherAgentCoordinates().begin();
        it != m_cAgVisLF.GetOtherAgentCoordinates().end();
        ++it){
        DrawCoordinates(it->second, CColor::BLUE);
    }


}

/****************************************/
/****************************************/

void CAgentVisionQTUserFunctions::DrawCoordinates(const std::vector<CVector3> &c_coordinates, CColor color) {
    /* Start drawing segments when you have at least two points */
    CQuaternion orientation = CQuaternion();
    for(CVector3 coordinate : c_coordinates){
        DrawCircle(coordinate, orientation, 0.1f, color);
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAgentVisionQTUserFunctions, "agent_vision_qtuser_functions_pipuck")
