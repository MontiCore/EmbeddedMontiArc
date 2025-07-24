#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_FOLLOWTRAJECTORY
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_BEHAVIORGENERATION_CALCMOTIONCMDS_FOLLOWTRAJECTORY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_behaviorGeneration_calcMotionCmds_followTrajectory{
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
double BEZIER_COEF;
double TURN_ANGLE_THRESHOLD;
double EPSILON;
double REACHED_POSITION_THRESHOLD;
void init()
{
trimmedTrajectoryX=Matrix(1,100);
trimmedTrajectoryY=Matrix(1,100);
BEZIER_COEF=0.1;
TURN_ANGLE_THRESHOLD=10*M_PI/180;
EPSILON=1.0E-5;
REACHED_POSITION_THRESHOLD=5;
}
void execute()
{
desiredDirectionX = 0;
desiredDirectionY = 0;
signedDistanceToTrajectory = 0;
minVelocity = 0;
maxVelocity = 0;
distance = 0;
if((isDriveToFirstPosition)){
double doNothing;
}
else if(((trimmedTrajectoryLength >= 2))){
maxVelocity = 7;
double p1x = trimmedTrajectoryX(1-1, 1-1);
double p1y = trimmedTrajectoryY(1-1, 1-1);
double p2x = trimmedTrajectoryX(1-1, 2-1);
double p2y = trimmedTrajectoryY(1-1, 2-1);
double v12x = p2x-p1x;
double v12y = p2y-p1y;
double v12_sqr_norm = v12x*v12x+v12y*v12y;
double v12_norm = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(v12_sqr_norm),1)));
if(((v12_norm > EPSILON))){
signedDistanceToTrajectory = -(v12y*currentPositionX-v12x*currentPositionY+p2x*p1y-p1x*p2y)/v12_norm;
}
double nextX = currentPositionX;
double nextY = currentPositionY;
if(((trimmedTrajectoryLength >= 3))){
double interp_p1x = trimmedTrajectoryX(1-1, 1-1);
double interp_p1y = trimmedTrajectoryY(1-1, 1-1);
double interp_p2x = trimmedTrajectoryX(1-1, 2-1);
double interp_p2y = trimmedTrajectoryY(1-1, 2-1);
double interp_p3x = trimmedTrajectoryX(1-1, 3-1);
double interp_p3y = trimmedTrajectoryY(1-1, 3-1);
double t = BEZIER_COEF;
double a = 1-t;
double b = t;
double k1 = a*a;
double k2 = 2*a*b;
double k3 = b*b;
nextX = k1*interp_p1x+k2*interp_p2x+k3*interp_p3x;
nextY = k1*interp_p1y+k2*interp_p2y+k3*interp_p3y;
double dist_to_next_sqr = (nextX-currentPositionX)*(nextX-currentPositionX)+(nextY-currentPositionY)*(nextY-currentPositionY);
double dist_to_next = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(dist_to_next_sqr),1)));
if(((dist_to_next <= REACHED_POSITION_THRESHOLD))){
double is_go_on_1 = 1;
for( auto j=2;j<=trimmedTrajectoryLength;++j){
if((is_go_on_1)){
double point_x = trimmedTrajectoryX(1-1, j-1);
double point_y = trimmedTrajectoryY(1-1, j-1);
double dist_to_point_sqr = (point_x-currentPositionX)*(point_x-currentPositionX)+(point_y-currentPositionY)*(point_y-currentPositionY);
double dist_to_point = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(dist_to_point_sqr),1)));
if(((dist_to_point > REACHED_POSITION_THRESHOLD))){
nextX = point_x;
nextY = point_y;
is_go_on_1 = 0;
}
}
}
if((is_go_on_1)){
nextX = trimmedTrajectoryX(1-1, trimmedTrajectoryLength-1);
nextY = trimmedTrajectoryY(1-1, trimmedTrajectoryLength-1);
}
}
}
else {
nextX = p2x;
nextY = p2y;
}
desiredDirectionX = nextX-currentPositionX;
desiredDirectionY = nextY-currentPositionY;
double dist = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList((p2x-p1x)*(p2x-p1x)+(p2y-p1y)*(p2y-p1y)),1)));
if(((trimmedTrajectoryLength >= 3))){
minVelocity = 3;
double is_go_on_2 = 1;
double lastIndex = trimmedTrajectoryLength-2;
for( auto i=1;i<=lastIndex;++i){
if((is_go_on_2)){
double pt1x = trimmedTrajectoryX(1-1, i-1);
double pt1y = trimmedTrajectoryY(1-1, i-1);
double pt2x = trimmedTrajectoryX(1-1, i+1-1);
double pt2y = trimmedTrajectoryY(1-1, i+1-1);
double pt3x = trimmedTrajectoryX(1-1, i+2-1);
double pt3y = trimmedTrajectoryY(1-1, i+2-1);
double vect1x = pt2x-pt1x;
double vect1y = pt2y-pt1y;
double vect2x = pt3x-pt2x;
double vect2y = pt3y-pt2y;
double alpha = 0;
double vect1_norm = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(vect1x*vect1x+vect1y*vect1y),1)));
double vect2_norm = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(vect2x*vect2x+vect2y*vect2y),1)));
if((((vect1_norm > EPSILON))&&((vect2_norm > EPSILON)))){
double cos_alpha = (vect1x*vect2x+vect1y*vect2y)/vect1_norm/vect2_norm;
alpha = (Helper::getDoubleFromOctaveListFirstResult(Facos(Helper::convertToOctaveValueList(cos_alpha),1)));
}
if(((alpha > TURN_ANGLE_THRESHOLD))){
is_go_on_2 = 0;
}
else {
dist += vect2_norm;
}
}
}
}
distance = dist;
}
}

};
#endif
