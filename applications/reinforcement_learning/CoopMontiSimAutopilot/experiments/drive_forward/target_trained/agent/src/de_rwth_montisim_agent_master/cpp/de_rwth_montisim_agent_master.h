#ifndef DE_RWTH_MONTISIM_AGENT_MASTER
#define DE_RWTH_MONTISIM_AGENT_MASTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "de_rwth_montisim_agent_master_qnet.h"
using namespace arma;
class de_rwth_montisim_agent_master{
public:
colvec state;
bool terminated;
colvec action;
bool terminate;
de_rwth_montisim_agent_master_qnet qnet;
void init()
{
state=colvec(25);
action=colvec(3);
qnet.init();
}
void execute()
{
qnet.state = state;
terminate = terminated;
qnet.execute();
action = qnet.action;
}

};
#endif
