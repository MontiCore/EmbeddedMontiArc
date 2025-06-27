/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS
#define TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_isDriveToFirstPosition.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_driveToFirstPosition.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_d2v1.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_followTrajectory.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_d2v2.h"
#include "testing_subpackage9_myComponent9_calcMotionCmds_selector.h"
class testing_subpackage9_myComponent9_calcMotionCmds{
public:
double currentPositionX;
double currentPositionY;
double currentDirectionX;
double currentDirectionY;
int trimmedTrajectoryLength;
Matrix trimmedTrajectoryX;
Matrix trimmedTrajectoryY;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double desiredVelocity;
testing_subpackage9_myComponent9_calcMotionCmds_isDriveToFirstPosition isDriveToFirstPosition;
testing_subpackage9_myComponent9_calcMotionCmds_driveToFirstPosition driveToFirstPosition;
testing_subpackage9_myComponent9_calcMotionCmds_d2v1 d2v1;
testing_subpackage9_myComponent9_calcMotionCmds_followTrajectory followTrajectory;
testing_subpackage9_myComponent9_calcMotionCmds_d2v2 d2v2;
testing_subpackage9_myComponent9_calcMotionCmds_selector selector;
void init()
{
trimmedTrajectoryX=Matrix(1,100);
trimmedTrajectoryY=Matrix(1,100);
isDriveToFirstPosition.init();
driveToFirstPosition.init();
d2v1.init();
followTrajectory.init();
d2v2.init();
selector.init();
}
void execute()
{
}

};
#endif
