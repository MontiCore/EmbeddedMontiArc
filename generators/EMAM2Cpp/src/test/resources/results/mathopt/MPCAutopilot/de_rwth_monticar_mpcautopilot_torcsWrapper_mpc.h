#ifndef DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_MPC
#define DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_MPC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "de_rwth_monticar_mpcautopilot_torcsWrapper_mpc_pFollow.h"
using namespace arma;
class de_rwth_monticar_mpcautopilot_torcsWrapper_mpc{
public:
double angleOnTrackAxis;
double distanceFromTrackAxis;
colvec currentSpeed;
colvec wheelSpeeds;
double steering;
double gasPedal;
double brakePedal;
de_rwth_monticar_mpcautopilot_torcsWrapper_mpc_pFollow pFollow;
void init()
{
currentSpeed=colvec(3);
wheelSpeeds=colvec(4);
pFollow.init();
}
void execute()
{
}

};
#endif
