#ifndef DETECTION_OBJECTDETECTOR2_SPECTRALCLUSTERER_1_
#define DETECTION_OBJECTDETECTOR2_SPECTRALCLUSTERER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "detection_objectDetector2_spectralClusterer_1__similarity.h"
#include "detection_objectDetector2_spectralClusterer_1__normalizedLaplacian.h"
#include "detection_objectDetector2_spectralClusterer_1__eigenSolver.h"
#include "detection_objectDetector2_spectralClusterer_1__kMeansClustering.h"
using namespace arma;
class detection_objectDetector2_spectralClusterer_1_{
const int n = 50;
const int elements = 2500;
const int k = 4;
const int maximumClusters = 1;
public:
mat red;
mat green;
mat blue;
mat clusters;
detection_objectDetector2_spectralClusterer_1__similarity similarity;
detection_objectDetector2_spectralClusterer_1__normalizedLaplacian normalizedLaplacian;
detection_objectDetector2_spectralClusterer_1__eigenSolver eigenSolver;
detection_objectDetector2_spectralClusterer_1__kMeansClustering kMeansClustering;
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
similarity.red = red;
similarity.green = green;
similarity.blue = blue;
similarity.execute();
normalizedLaplacian.degree = similarity.degree;
normalizedLaplacian.similarity = similarity.similarity;
normalizedLaplacian.execute();
eigenSolver.matrix = normalizedLaplacian.nLaplacian;
eigenSolver.execute();
kMeansClustering.vectors = eigenSolver.eigenvectors;
kMeansClustering.execute();
clusters = kMeansClustering.clusters;
}

};
#endif
