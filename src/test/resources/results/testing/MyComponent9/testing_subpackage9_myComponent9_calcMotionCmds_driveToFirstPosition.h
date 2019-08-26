/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_DRIVETOFIRSTPOSITION
#define TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_DRIVETOFIRSTPOSITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage9_myComponent9_calcMotionCmds_driveToFirstPosition{
public:
bool isDriveToFirstPosition;
double currentPositionX;
double currentPositionY;
int trimmedTrajectoryLength;
Matrix trimmedTrajectoryX;
Matrix trimmedTrajectoryY;
double desiredDirectionX;
double desiredDirectionY;
double signedDistanceToTrajectory;
double minVelocity;
double maxVelocity;
double distance;
void init()
{
trimmedTrajectoryX=Matrix(1,100);
trimmedTrajectoryY=Matrix(1,100);
}
void execute()
{
desiredDirectionX = 0;
desiredDirectionY = 0;
signedDistanceToTrajectory = 0;
minVelocity = 0;
maxVelocity = 0;
distance = 0;
if((isDriveToFirstPosition)&&((trimmedTrajectoryLength >= 1))){
maxVelocity = 7;
desiredDirectionX = trimmedTrajectoryX(1-1, 1-1)-currentPositionX;
desiredDirectionY = trimmedTrajectoryY(1-1, 1-1)-currentPositionY;
double sqr_dist = desiredDirectionX*desiredDirectionX+desiredDirectionY*desiredDirectionY;
distance = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(sqr_dist),1)));
if(((trimmedTrajectoryLength >= 2))){
minVelocity = 3;
}
}
}

};
#endif
