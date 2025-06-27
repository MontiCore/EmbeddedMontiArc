/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSSTEERING
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_BOUNDSSTEERING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_boundsSteering_eb.h"
class testing_subpackage10_myComponent10_motionPlanning_boundsSteering{
public:
double input;
double output;
double CONSTANTPORT1;
double CONSTANTPORT2;
testing_subpackage10_myComponent10_motionPlanning_boundsSteering_eb eb;
void init()
{
this->CONSTANTPORT1 = -0.785;
this->CONSTANTPORT2 = 0.785;
eb.init();
}
void execute()
{
}

};
#endif
