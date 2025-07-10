/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_SELECTOR
#define TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_SELECTOR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_subpackage9_myComponent9_calcMotionCmds_selector{
public:
bool isDriveToFirstPosition;
double desiredDirectionX1;
double desiredDirectionY1;
double signedDistanceToTrajectory1;
double desiredVelocity1;
double desiredDirectionX2;
double desiredDirectionY2;
double signedDistanceToTrajectory2;
double desiredVelocity2;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double desiredVelocity;
void init()
{
}
void execute()
{
if((isDriveToFirstPosition)){
desiredDirectionX = desiredDirectionX1;
desiredDirectionY = desiredDirectionY1;
signedDistanceToTrajectory = signedDistanceToTrajectory1;
desiredVelocity = desiredVelocity1;
}
else {
desiredDirectionX = desiredDirectionX2;
desiredDirectionY = desiredDirectionY2;
signedDistanceToTrajectory = signedDistanceToTrajectory2;
desiredVelocity = desiredVelocity2;
}
}

};
#endif
