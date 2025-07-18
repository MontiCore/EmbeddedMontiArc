/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSENGINE
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSENGINE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsEngine_eb.h"
class testing_subpackage10_myComponent10_motionPlanning_boundsEngine{
public:
double input;
double output;
double CONSTANTPORT5;
double CONSTANTPORT6;
testing_subpackage10_myComponent10_motionPlanning_boundsEngine_eb eb;
void init()
{
this->CONSTANTPORT5 = 0;
this->CONSTANTPORT6 = 2.5;
eb.init();
}
void execute()
{
}

};
#endif
