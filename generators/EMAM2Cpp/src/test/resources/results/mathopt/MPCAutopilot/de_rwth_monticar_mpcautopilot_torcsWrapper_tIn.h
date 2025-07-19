#ifndef DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_TIN
#define DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_TIN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class de_rwth_monticar_mpcautopilot_torcsWrapper_tIn{
public:
colvec state;
double angleOnTrackAxis;
double trackPosition;
colvec speedVector;
colvec wheelSpeeds;
void init()
{
state=colvec(29);
speedVector=colvec(3);
wheelSpeeds=colvec(4);
}
void execute()
{
angleOnTrackAxis = state(1-1);
trackPosition = state(21-1);
speedVector(1-1) = state(22-1);
speedVector(2-1) = state(23-1);
speedVector(3-1) = state(24-1);
wheelSpeeds(1-1) = state(25-1);
wheelSpeeds(2-1) = state(26-1);
wheelSpeeds(3-1) = state(27-1);
wheelSpeeds(4-1) = state(28-1);
}

};
#endif
