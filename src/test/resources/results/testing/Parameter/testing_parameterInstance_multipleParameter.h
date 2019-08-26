/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_PARAMETERINSTANCE_MULTIPLEPARAMETER
#define TESTING_PARAMETERINSTANCE_MULTIPLEPARAMETER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_parameterInstance_multipleParameter{
public:
double param1;
double param2;
double param3;
void init(double param1, double param2, double param3)
{
this->param1=param1;
this->param2=param2;
this->param3=param3;
}
void execute()
{
}

};
#endif
