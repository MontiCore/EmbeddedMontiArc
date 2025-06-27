/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_PARAMETERINSTANCE_SINGLEPARAMETER
#define TESTING_PARAMETERINSTANCE_SINGLEPARAMETER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_parameterInstance_singleParameter{
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
