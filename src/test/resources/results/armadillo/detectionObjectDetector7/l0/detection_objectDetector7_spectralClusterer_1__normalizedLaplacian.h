/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR7_SPECTRALCLUSTERER_1__NORMALIZEDLAPLACIAN
#define DETECTION_OBJECTDETECTOR7_SPECTRALCLUSTERER_1__NORMALIZEDLAPLACIAN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "HelperA.h"
using namespace arma;
class detection_objectDetector7_spectralClusterer_1__normalizedLaplacian{
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
nLaplacian = (HelperA::getSqrtMat((inv(degree))))*similarity*(HelperA::getSqrtMat((inv(degree))));
}

};
#endif
