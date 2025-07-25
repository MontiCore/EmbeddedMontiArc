/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER_NORMALIZEDLAPLACIAN
#define DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER_NORMALIZEDLAPLACIAN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
using namespace arma;
class detection_objectDetector1_spectralClusterer_normalizedLaplacian{
const int n = 2500;
public:
mat degree;
mat similarity;
mat nLaplacian;
void init()
{
degree=mat(n,n);
similarity=mat(n,n);
nLaplacian=mat(n,n);
}
void execute()
{
auto _I_0 = (HelperA::getSqrtMat((inv(degree))));
nLaplacian = _I_0*similarity*_I_0;
}

};
#endif
