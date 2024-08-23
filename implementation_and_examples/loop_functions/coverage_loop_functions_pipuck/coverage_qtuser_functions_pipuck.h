#ifndef COVERAGE_QTUSER_FUNCTIONS_PIPUCK_H
#define COVERAGE_QTUSER_FUNCTIONS_PIPUCK_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>

using namespace argos;

class CCoverageLoopFunctions;

class CCoverageQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CCoverageQTUserFunctions();

   virtual ~CCoverageQTUserFunctions() {}

   virtual void DrawInWorld();

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CCoverageLoopFunctions& m_cTrajLF;

};

#endif
