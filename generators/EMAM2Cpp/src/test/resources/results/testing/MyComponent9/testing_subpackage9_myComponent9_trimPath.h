/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE9_MYCOMPONENT9_TRIMPATH
#define TESTING_SUBPACKAGE9_MYCOMPONENT9_TRIMPATH
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage9_myComponent9_trimPath{
public:
double currentPositionX;
double currentPositionY;
int plannedTrajectoryLength;
Matrix plannedTrajectoryX;
Matrix plannedTrajectoryY;
int trimmedTrajectoryLength;
Matrix trimmedTrajectoryX;
Matrix trimmedTrajectoryY;
void init()
{
plannedTrajectoryX=Matrix(1,100);
plannedTrajectoryY=Matrix(1,100);
trimmedTrajectoryX=Matrix(1,100);
trimmedTrajectoryY=Matrix(1,100);
}
void execute()
{
trimmedTrajectoryLength = 0;
if((plannedTrajectoryLength == 1)){
trimmedTrajectoryLength = 1;
trimmedTrajectoryX(1-1, 1-1) = plannedTrajectoryX(1-1, 1-1);
trimmedTrajectoryY(1-1, 1-1) = plannedTrajectoryY(1-1, 1-1);
}
else if((plannedTrajectoryLength > 1)){
double closestSegmentIndex = -1;
double closestSegmentDistance = -1;
double lastSegmentIndex = plannedTrajectoryLength-1;
for( auto i=1;i<=lastSegmentIndex;++i){
double p1x = plannedTrajectoryX(1-1, i-1);
double p1y = plannedTrajectoryY(1-1, i-1);
double p2x = plannedTrajectoryX(1-1, i+1-1);
double p2y = plannedTrajectoryY(1-1, i+1-1);
double vx = currentPositionX-p1x;
double vy = currentPositionY-p1y;
double v12x = p2x-p1x;
double v12y = p2y-p1y;
double k = (vx*v12x+vy*v12y)/(v12x*v12x+v12y*v12y);
double projection_x = p1x+k*v12x;
double projection_y = p1y+k*v12y;
double is_projection_on_segment = (((p1x-projection_x)*(p2x-projection_x) <= 0))&&(((p1y-projection_y)*(p2y-projection_y) <= 0));
if((is_projection_on_segment)){
double d_proj_sqr = (currentPositionX-projection_x)*(currentPositionX-projection_x)+(currentPositionY-projection_y)*(currentPositionY-projection_y);
double d_proj = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(d_proj_sqr),1)));
if(((closestSegmentDistance < 0))||((d_proj < closestSegmentDistance))){
closestSegmentIndex = i;
closestSegmentDistance = d_proj;
trimmedTrajectoryX(1-1, 1-1) = projection_x;
trimmedTrajectoryY(1-1, 1-1) = projection_y;
}
}
else {
double d1_sqr = (currentPositionX-p1x)*(currentPositionX-p1x)+(currentPositionY-p1y)*(currentPositionY-p1y);
double d1 = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(d1_sqr),1)));
double d2_sqr = (currentPositionX-p2x)*(currentPositionX-p2x)+(currentPositionY-p2y)*(currentPositionY-p2y);
double d2 = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(d2_sqr),1)));
double d_min = (Helper::getDoubleFromOctaveListFirstResult(Fmin(Helper::convertToOctaveValueList(d1, d2),1)));
if(((closestSegmentDistance < 0))||((d_min < closestSegmentDistance))){
closestSegmentIndex = i;
closestSegmentDistance = d_min;
trimmedTrajectoryX(1-1, 1-1) = projection_x;
trimmedTrajectoryY(1-1, 1-1) = projection_y;
}
}
}
if((closestSegmentIndex > -1)){
double currentFree = 2;
double start = closestSegmentIndex+1;
for( auto i=start;i<=plannedTrajectoryLength;++i){
trimmedTrajectoryX(1-1, currentFree-1) = plannedTrajectoryX(1-1, i-1);
trimmedTrajectoryY(1-1, currentFree-1) = plannedTrajectoryY(1-1, i-1);
currentFree += 1;
}
trimmedTrajectoryLength = currentFree-1;
}
}
}

};
#endif
