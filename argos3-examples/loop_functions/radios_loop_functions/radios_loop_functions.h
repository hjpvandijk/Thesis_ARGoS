#ifndef TEST_LOOP_FUNCTIONS_H
#define TEST_LOOP_FUNCTIONS_H

namespace argos {
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>

namespace argos {

   class CRadiosLoopFunctions : public CLoopFunctions {

   public:

      CRadiosLoopFunctions() {}

      virtual ~CRadiosLoopFunctions() {}

//      virtual bool IsExperimentFinished() override;
       virtual void PostStep();
   };
}

#endif

