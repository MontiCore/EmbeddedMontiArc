%module any_component_executor

%{
#define SWIG_FILE_WITH_INIT
#include "any_component_executor.h"
%}

%include "armanpy/armanpy.i"
%include "any_component_executor.h"
