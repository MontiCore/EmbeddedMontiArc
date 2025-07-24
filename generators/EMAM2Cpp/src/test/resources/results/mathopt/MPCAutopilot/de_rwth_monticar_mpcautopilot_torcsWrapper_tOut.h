#ifndef DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_TOUT
#define DE_RWTH_MONTICAR_MPCAUTOPILOT_TORCSWRAPPER_TOUT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class de_rwth_monticar_mpcautopilot_torcsWrapper_tOut{
public:
double steering;
double gasPedal;
double brakePedal;
colvec action;
void init()
{
action=colvec(3);
}
void execute()
{
action(1-1) = steering;
action(2-1) = (gasPedal-500)/500;
action(3-1) = (brakePedal-500)/500;
}

};
#endif
