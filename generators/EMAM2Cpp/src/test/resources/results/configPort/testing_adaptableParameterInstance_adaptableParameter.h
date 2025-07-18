/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_ADAPTABLEPARAMETERINSTANCE_ADAPTABLEPARAMETER
#define TESTING_ADAPTABLEPARAMETERINSTANCE_ADAPTABLEPARAMETER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_adaptableParameterInstance_adaptableParameter{
public:
double param1;
double param2;
double out1;
void init(double param1, double param2)
{
this->param1=param1;
this->param2=param2;
}
void execute()
{
out1 = param1;
}

};
#endif
