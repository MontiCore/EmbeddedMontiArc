/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_KEEPDIRECTION
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_KEEPDIRECTION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_keepDirection_sab.h"
class testing_subpackage10_myComponent10_motionPlanning_keepDirection{
public:
double currentDirectionX;
double currentDirectionY;
double desiredDirectionX;
double desiredDirectionY;
double steeringAngle;
testing_subpackage10_myComponent10_motionPlanning_keepDirection_sab sab;
void init()
{
sab.init();
}
void execute()
{
}

};
#endif
