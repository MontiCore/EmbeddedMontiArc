/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_ISDRIVETOFIRSTPOSITION
#define TESTING_SUBPACKAGE9_MYCOMPONENT9_CALCMOTIONCMDS_ISDRIVETOFIRSTPOSITION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage9_myComponent9_calcMotionCmds_isDriveToFirstPosition{
public:
double currentPositionX;
double currentPositionY;
int trimmedTrajectoryLength;
Matrix trimmedTrajectoryX;
Matrix trimmedTrajectoryY;
bool result;
void init()
{
trimmedTrajectoryX=Matrix(1,100);
trimmedTrajectoryY=Matrix(1,100);
}
void execute()
{
double TOO_FAR_FROM_TRAJECTORY_THRESHOLD = 3;
result = 1;
if((trimmedTrajectoryLength > 0)){
double dx = trimmedTrajectoryX(1-1, 1-1)-currentPositionX;
double dy = trimmedTrajectoryY(1-1, 1-1)-currentPositionY;
double sqr_dist = dx*dx+dy*dy;
double dist_to_trajectory = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(sqr_dist),1)));
double is_vehicle_went_off_trajectory = (dist_to_trajectory > TOO_FAR_FROM_TRAJECTORY_THRESHOLD);
double is_only_one_position_specified = (trimmedTrajectoryLength <= 1);
result = is_vehicle_went_off_trajectory||is_only_one_position_specified;
}
}

};
#endif
