/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSENGINE_EB
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSENGINE_EB
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_subpackage10_myComponent10_motionPlanning_boundsEngine_eb{
public:
double lowerBound;
double upperBound;
double input;
double output;
void init()
{
}
void execute()
{
if((input < lowerBound)){
output = lowerBound;
}
else if((input > upperBound)){
output = upperBound;
}
else {
output = input;
}
}

};
#endif
