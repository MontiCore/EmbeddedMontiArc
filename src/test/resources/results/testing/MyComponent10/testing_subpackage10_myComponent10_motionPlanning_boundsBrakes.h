/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSBRAKES
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSBRAKES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsBrakes_eb.h"
class testing_subpackage10_myComponent10_motionPlanning_boundsBrakes{
public:
double input;
double output;
double CONSTANTPORT3;
double CONSTANTPORT4;
testing_subpackage10_myComponent10_motionPlanning_boundsBrakes_eb eb;
void init()
{
this->CONSTANTPORT3 = 0;
this->CONSTANTPORT4 = 3;
eb.init();
}
void execute()
{
}

};
#endif
