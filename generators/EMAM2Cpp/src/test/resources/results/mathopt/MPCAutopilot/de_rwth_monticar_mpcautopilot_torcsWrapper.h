#ifndef DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER
#define DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "de_rwth_monticar_mpcautopilot_torcsWrapper_tIn.h"
#include "de_rwth_monticar_mpcautopilot_torcsWrapper_mpc.h"
#include "de_rwth_monticar_mpcautopilot_torcsWrapper_tOut.h"
using namespace arma;
class de_rwth_monticar_mpcautopilot_torcsWrapper{
public:
colvec state;
colvec action;
de_rwth_monticar_mpcautopilot_torcsWrapper_tIn tIn;
de_rwth_monticar_mpcautopilot_torcsWrapper_mpc mpc;
de_rwth_monticar_mpcautopilot_torcsWrapper_tOut tOut;
void init()
{
state=colvec(29);
action=colvec(3);
tIn.init();
mpc.init();
tOut.init();
}
void execute()
{
tIn.state = state;
tIn.execute();
mpc.pFollow.angleOnTrackAxis = tIn.angleOnTrackAxis;
mpc.pFollow.distanceFromTrackAxis = tIn.trackPosition;
mpc.pFollow.currentSpeed = tIn.speedVector;
mpc.pFollow.wheelSpeeds = tIn.wheelSpeeds;
mpc.pFollow.execute();
tOut.steering = mpc.pFollow.steering;
tOut.gasPedal = mpc.pFollow.gasPedal;
tOut.brakePedal = mpc.pFollow.brakePedal;
tOut.execute();
action = tOut.action;
}

};
#endif
