/* (c) https://github.com/MontiCore/monticore */
#ifndef DETECTION_OBJECTDETECTOR6_SPECTRALCLUSTERER_1_
#define DETECTION_OBJECTDETECTOR6_SPECTRALCLUSTERER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "detection_objectDetector6_spectralClusterer_1__similarity.h"
#include "detection_objectDetector6_spectralClusterer_1__normalizedLaplacian.h"
#include "detection_objectDetector6_spectralClusterer_1__eigenSolver.h"
#include "detection_objectDetector6_spectralClusterer_1__kMeansClustering.h"
using namespace arma;
class detection_objectDetector6_spectralClusterer_1_{
const int n = 50;
const int elements = 2500;
const int k = 4;
const int maximumClusters = 1;
public:
mat red;
mat green;
mat blue;
mat clusters;
detection_objectDetector6_spectralClusterer_1__similarity similarity;
detection_objectDetector6_spectralClusterer_1__normalizedLaplacian normalizedLaplacian;
detection_objectDetector6_spectralClusterer_1__eigenSolver eigenSolver;
detection_objectDetector6_spectralClusterer_1__kMeansClustering kMeansClustering;
void init()
{
red=mat(n,n);
green=mat(n,n);
blue=mat(n,n);
clusters=mat(elements,maximumClusters);
similarity.init();
normalizedLaplacian.init();
eigenSolver.init();
kMeansClustering.init();
}
void execute()
{
}

};
#endif
