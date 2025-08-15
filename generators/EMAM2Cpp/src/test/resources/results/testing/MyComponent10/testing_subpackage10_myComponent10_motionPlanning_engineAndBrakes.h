/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_ENGINEANDBRAKES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pidParams.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pidError.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_abs1.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pid.h"
#include "testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_engineOrBrakes.h"
class testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes{
public:
double currentVelocity;
double desiredVelocity;
double engine;
double brakes;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pidParams pidParams;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pidError pidError;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_abs1 abs1;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_pid pid;
testing_subpackage10_myComponent10_motionPlanning_engineAndBrakes_engineOrBrakes engineOrBrakes;
void init()
{
pidParams.init();
pidError.init();
abs1.init();
pid.init();
engineOrBrakes.init();
}
void execute()
{
}

};
#endif
