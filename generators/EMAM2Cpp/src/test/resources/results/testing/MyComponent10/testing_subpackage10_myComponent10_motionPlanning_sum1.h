/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_SUM1
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_SUM1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_subpackage10_myComponent10_motionPlanning_sum1{
public:
double t1;
double t2;
double result;
void init()
{
}
void execute()
{
result = t1+t2;
}

};
#endif
