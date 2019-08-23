/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_ADAPTABLEPARAMETERINSTANCE_ADAPTABLEPARAMETER2
#define TESTING_ADAPTABLEPARAMETERINSTANCE_ADAPTABLEPARAMETER2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_adaptableParameterInstance_adaptableParameter2{
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
