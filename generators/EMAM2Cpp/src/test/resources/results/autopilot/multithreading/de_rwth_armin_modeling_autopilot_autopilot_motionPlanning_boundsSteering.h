#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_BOUNDSSTEERING
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_BOUNDSSTEERING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsSteering_eb.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsSteering{
public:
double input;
double output;
double CONSTANTPORT1;
double CONSTANTPORT2;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_boundsSteering_eb eb;
void init()
{
this->CONSTANTPORT1 = -0.785;
this->CONSTANTPORT2 = 0.785;
eb.init();
}
void execute()
{
}

};
#endif
