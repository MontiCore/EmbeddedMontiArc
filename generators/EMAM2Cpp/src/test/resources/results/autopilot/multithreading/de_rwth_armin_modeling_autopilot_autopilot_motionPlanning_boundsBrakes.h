#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_BOUNDSBRAKES
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_BOUNDSBRAKES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsBrakes_eb.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsBrakes{
public:
double input;
double output;
double CONSTANTPORT3;
double CONSTANTPORT4;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsBrakes_eb eb;
void init()
{
this->CONSTANTPORT3 = 0;
this->CONSTANTPORT4 = 3;
eb.init();
}
void execute()
{
}

};
#endif
