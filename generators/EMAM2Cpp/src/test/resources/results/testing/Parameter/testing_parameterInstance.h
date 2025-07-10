/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_PARAMETERINSTANCE
#define TESTING_PARAMETERINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_parameterInstance_singleParameter.h"
#include "testing_parameterInstance_singleParameter2.h"
#include "testing_parameterInstance_multipleParameter.h"
#include "testing_parameterInstance_multipleParameter2.h"
class testing_parameterInstance{
public:
testing_parameterInstance_singleParameter singleParameter;
testing_parameterInstance_singleParameter2 singleParameter2;
testing_parameterInstance_multipleParameter multipleParameter;
testing_parameterInstance_multipleParameter2 multipleParameter2;
void init()
{
singleParameter.init(1.2);
singleParameter2.init(2);
multipleParameter.init(1, 1.4, 2);
multipleParameter2.init(3, 4, 5);
}
void execute()
{
singleParameter.execute();
singleParameter2.execute();
multipleParameter.execute();
multipleParameter2.execute();
}

};
#endif
