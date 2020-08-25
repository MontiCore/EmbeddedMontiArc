/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_PARAMETERINSTANCE_SINGLEPARAMETER2
#define TESTING_PARAMETERINSTANCE_SINGLEPARAMETER2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_parameterInstance_singleParameter2{
public:
double param1;
void init(double param1)
{
this->param1=param1;
}
void execute()
{
}

};
#endif
