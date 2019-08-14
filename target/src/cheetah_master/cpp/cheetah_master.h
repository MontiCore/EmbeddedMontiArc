#ifndef CHEETAH_MASTER
#define CHEETAH_MASTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "cheetah_master_cheetah.h"
using namespace arma;
class cheetah_master{
public:
colvec state;
colvec action;
cheetah_master_cheetah cheetah;
void init()
{
state=colvec(26);
action=colvec(6);
cheetah.init();
}
void execute()
{
cheetah.state = state;
cheetah.execute();
action = cheetah.action;
}

};
#endif
