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
double trimmedTrajectoryLength;
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
isDriveToFirstPosition.currentPositionX = currentPositionX;
isDriveToFirstPosition.currentPositionY = currentPositionY;
isDriveToFirstPosition.trimmedTrajectoryLength = trimmedTrajectoryLength;
isDriveToFirstPosition.trimmedTrajectoryX = trimmedTrajectoryX;
isDriveToFirstPosition.trimmedTrajectoryY = trimmedTrajectoryY;
isDriveToFirstPosition.execute();
driveToFirstPosition.isDriveToFirstPosition = isDriveToFirstPosition.result;
driveToFirstPosition.currentPositionX = currentPositionX;
driveToFirstPosition.currentPositionY = currentPositionY;
driveToFirstPosition.trimmedTrajectoryLength = trimmedTrajectoryLength;
driveToFirstPosition.trimmedTrajectoryX = trimmedTrajectoryX;
driveToFirstPosition.trimmedTrajectoryY = trimmedTrajectoryY;
driveToFirstPosition.execute();
d2v1.distance = driveToFirstPosition.distance;
d2v1.minVelocity = driveToFirstPosition.minVelocity;
d2v1.maxVelocity = driveToFirstPosition.maxVelocity;
d2v1.execute();
followTrajectory.isDriveToFirstPosition = isDriveToFirstPosition.result;
followTrajectory.currentPositionX = currentPositionX;
followTrajectory.currentPositionY = currentPositionY;
followTrajectory.trimmedTrajectoryLength = trimmedTrajectoryLength;
followTrajectory.trimmedTrajectoryX = trimmedTrajectoryX;
followTrajectory.trimmedTrajectoryY = trimmedTrajectoryY;
followTrajectory.execute();
d2v2.distance = followTrajectory.distance;
d2v2.minVelocity = followTrajectory.minVelocity;
d2v2.maxVelocity = followTrajectory.maxVelocity;
d2v2.execute();
selector.isDriveToFirstPosition = isDriveToFirstPosition.result;
selector.desiredDirectionX1 = driveToFirstPosition.desiredDirectionX;
selector.desiredDirectionY1 = driveToFirstPosition.desiredDirectionY;
selector.signedDistanceToTrajectory1 = driveToFirstPosition.signedDistanceToTrajectory;
selector.desiredVelocity1 = d2v1.velocity;
selector.desiredDirectionX2 = followTrajectory.desiredDirectionX;
selector.desiredDirectionY2 = followTrajectory.desiredDirectionY;
selector.signedDistanceToTrajectory2 = followTrajectory.signedDistanceToTrajectory;
selector.desiredVelocity2 = d2v2.velocity;
selector.execute();
desiredDirectionX = selector.desiredDirectionX;
desiredDirectionY = selector.desiredDirectionY;
signedDistanceToTrajectory = selector.signedDistanceToTrajectory;
desiredVelocity = selector.desiredVelocity;
}

};
#endif
