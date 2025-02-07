#ifndef AGENT_VISION_QTUSER_FUNCTIONS_PIPUCK_H
#define AGENT_VISION_QTUSER_FUNCTIONS_PIPUCK_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include "agent_implementation/utils/coordinate.h"
#include "agent_implementation/feature_config.h"

using namespace argos;

class CAgentVisionLoopFunctions;

class CAgentVisionQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CAgentVisionQTUserFunctions();

   virtual ~CAgentVisionQTUserFunctions() {}

   virtual void DrawInWorld();

private:

   void DrawCoordinates(const std::vector<CVector3>& c_coordinates, CColor color);

private:

   CAgentVisionLoopFunctions& m_cAgVisLF;

};

#endif
