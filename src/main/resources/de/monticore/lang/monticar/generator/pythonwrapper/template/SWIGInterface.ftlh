<#-- (c) https://github.com/MontiCore/monticore -->
%module ${wrapperClassName}

%{
#define SWIG_FILE_WITH_INIT
#include "${wrapperClassName}.h"
%}

%include "armanpy/armanpy.i"
%include "${wrapperClassName}.h"
