#ifndef MODELS_SCALARPORT
#define MODELS_SCALARPORT
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class models_scalarPort{
public:
bool in1;
bool out1;
void init()
{
}
void execute()
{
out1 = in1;
}

};
#endif
