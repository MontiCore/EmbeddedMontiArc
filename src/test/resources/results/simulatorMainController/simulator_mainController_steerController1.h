/* (c) https://github.com/MontiCore/monticore */
#ifndef SIMULATOR_MAINCONTROLLER_STEERCONTROLLER1
#define SIMULATOR_MAINCONTROLLER_STEERCONTROLLER1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "simulator_mainController_steerController1_steeringCalculator.h"
class simulator_mainController_steerController1{
public:
double x[2];
double y[2];
double gpsX;
double gpsY;
double orientation;
double currentSteeringAngle;
double minSteeringAngle;
double maxSteeringAngle;
double steeringAngle;
simulator_mainController_steerController1_steeringCalculator steeringCalculator;
void init()
{
steeringCalculator.init();
}
void execute()
{
steeringCalculator.x[0] = x[0];
steeringCalculator.x[1] = x[1];
steeringCalculator.y[0] = y[0];
steeringCalculator.y[1] = y[1];
steeringCalculator.gpsX = gpsX;
steeringCalculator.gpsY = gpsY;
steeringCalculator.orientation = orientation;
steeringCalculator.minSteeringAngle = minSteeringAngle;
steeringCalculator.maxSteeringAngle = maxSteeringAngle;
steeringCalculator.currentSteeringAngle = currentSteeringAngle;
steeringCalculator.execute();
steeringAngle = steeringCalculator.newSteeringAngle;
}

};
#endif
