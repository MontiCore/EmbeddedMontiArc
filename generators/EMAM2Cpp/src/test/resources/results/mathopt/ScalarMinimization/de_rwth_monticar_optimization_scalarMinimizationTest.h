#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMINIMIZATIONTEST
#define DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMINIMIZATIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_scalarMinimizationTest{
public:
double xOut;
double yOut;
void init()
{
}
void execute()
{
double x;
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut);
xOut = x;
yOut = y;
}

};
#endif
