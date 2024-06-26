#ifndef TRAJECTORY_QTUSER_FUNCTIONS_PIPUCK_H
#define TRAJECTORY_QTUSER_FUNCTIONS_PIPUCK_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>

using namespace argos;

class CTrajectoryLoopFunctions;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTrajectoryQTUserFunctions();

   virtual ~CTrajectoryQTUserFunctions() {}

   virtual void DrawInWorld();

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CTrajectoryLoopFunctions& m_cTrajLF;

};

#endif
