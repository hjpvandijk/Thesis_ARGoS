#ifndef POSITION_LOOP_FUNCTIONS_LUA_H
#define POSITION_LOOP_FUNCTIONS_LUA_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CPositionLoopFunctions : public CLoopFunctions {

public:

   CPositionLoopFunctions();
   virtual ~CPositionLoopFunctions() {}

   virtual void PreStep();

private:
};

#endif
