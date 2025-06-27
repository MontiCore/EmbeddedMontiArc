/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES_ENGINEORBRAKES
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES_ENGINEORBRAKES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_engineOrBrakes{
public:
double error;
double controlSignal;
double engine;
double brakes;
void init()
{
}
void execute()
{
engine = 0;
brakes = 0;
if(((error > 0))){
engine = controlSignal;
}
else {
brakes = controlSignal;
}
}

};
#endif
