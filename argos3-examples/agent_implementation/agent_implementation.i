// agent.i
%module agent_implementation
%include "std_string.i"  // Add this line
%{
#include "agent.h"
#include "coordinate.h"
#include "radio.h"
//#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
using namespace argos;
%}

%include "agent.h"
%include "coordinate.h"
%include "radio.h"
%include <argos3/core/utility/math/vector2.h>
%include <argos3/core/utility/math/vector3.h>
