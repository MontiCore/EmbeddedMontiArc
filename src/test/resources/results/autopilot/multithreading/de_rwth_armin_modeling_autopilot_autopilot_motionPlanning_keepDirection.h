#ifndef DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_KEEPDIRECTION
#define DE_RWTH_ARMIN_MODELING_AUTOPILOT_AUTOPILOT_MOTIONPLANNING_KEEPDIRECTION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection_sab.h"
#include <thread>
class de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection{
public:
double currentDirectionX;
double currentDirectionY;
double desiredDirectionX;
double desiredDirectionY;
double steeringAngle;
de_rwth_armin_modeling_autopilot_autopilot_motionPlanning_keepDirection_sab sab;
void init()
{
sab.init();
}
void execute()
{
}

};
#endif
