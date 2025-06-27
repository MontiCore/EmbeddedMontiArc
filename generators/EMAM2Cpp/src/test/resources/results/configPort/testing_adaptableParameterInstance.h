/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_ADAPTABLEPARAMETERINSTANCE
#define TESTING_ADAPTABLEPARAMETERINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_adaptableParameterInstance_adaptableParameter.h"
#include "testing_adaptableParameterInstance_adaptableParameter2.h"
class testing_adaptableParameterInstance{
public:
double in1;
double out1;
double out2;
testing_adaptableParameterInstance_adaptableParameter adaptableParameter;
testing_adaptableParameterInstance_adaptableParameter2 adaptableParameter2;
void init()
{
adaptableParameter.init(1, 2);
adaptableParameter2.init(3, 4);
}
void execute()
{
adaptableParameter.param1 = in1;
adaptableParameter.execute();
out1 = adaptableParameter.out1;
adaptableParameter2.execute();
out2 = adaptableParameter2.out1;
}

};
#endif
