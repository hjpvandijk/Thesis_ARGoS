// agent.i
%module agent_implementation
%include "std_string.i"  // Add this line
%{
#include "agent.h"
#include "coordinate.h"
#include "radio.h"
%}

%include "agent.h"
%include "coordinate.h"
%include "radio.h"
