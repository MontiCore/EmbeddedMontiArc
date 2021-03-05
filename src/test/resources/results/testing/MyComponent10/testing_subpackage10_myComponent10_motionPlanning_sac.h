/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_SAC
#define TESTING_SUBPACKAGE10_MYCOMPONENT10_MOTIONPLANNING_SAC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
class testing_subpackage10_myComponent10_motionPlanning_sac{
public:
double signedDistanceToTrajectory;
double steeringAngleCorrection;
double MAX_STEERING_ANGLE;
double EPSILON;
double X1;
double Y1;
double X2;
double Y2;
double COEF_K;
void init()
{
MAX_STEERING_ANGLE=0.785;
EPSILON=0.01;
X1=0.1;
Y1=0.01*MAX_STEERING_ANGLE;
X2=5;
Y2=0.05*MAX_STEERING_ANGLE;
COEF_K=(Helper::getDoubleFromOctaveListFirstResult(Flog(Helper::convertToOctaveValueList((Y1/Y2)),1)))/(X2-X1);
}
void execute()
{
steeringAngleCorrection = 0;
double dist = (Helper::getDoubleFromOctaveListFirstResult(Fabs(Helper::convertToOctaveValueList(signedDistanceToTrajectory),1)));
if(((dist > EPSILON))){
if(((dist < X1))){
steeringAngleCorrection = Y1;
}
else if(((dist > X2))){
steeringAngleCorrection = Y2;
}
else {
steeringAngleCorrection = Y1*(Helper::getDoubleFromOctaveListFirstResult(Fexp(Helper::convertToOctaveValueList(-COEF_K*(dist-X1)),1)));
}
if(((signedDistanceToTrajectory < 0))){
steeringAngleCorrection *= -1;
}
}
}

};
#endif
