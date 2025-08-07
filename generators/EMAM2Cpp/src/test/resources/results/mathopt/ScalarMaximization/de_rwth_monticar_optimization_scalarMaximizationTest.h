#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMAXIMIZATIONTEST
#define DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMAXIMIZATIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt122.h"
using namespace arma;
class de_rwth_monticar_optimization_scalarMaximizationTest{
public:
double xOut;
double yOut;
void init()
{
}
void execute()
{
double x;
double z;
CallIpopt122::solveOptimizationProblemIpOpt(&x, &z, xOut, yOut);
yOut = z;
xOut = x;
}

};
#endif
